package main

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"log"
	"math/rand"
	"net"
	"os"
	"time"

	"github.com/function61/gokit/app/dynversion"
	. "github.com/function61/gokit/builtin"
	"github.com/function61/gokit/log/logex"
	"github.com/function61/gokit/os/osutil"
	"github.com/function61/gokit/sync/taskrunner"
	"github.com/spf13/cobra"
)

func main() {
	app := &cobra.Command{
		Use:     os.Args[0],
		Short:   "Mocap-pose but more robust",
		Version: dynversion.Version,
	}

	droneDeviceID := "sad04"
	if fromEnv := os.Getenv("DRONE_DEVICE_ID"); fromEnv != "" {
		droneDeviceID = fromEnv
	}

	cmd := &cobra.Command{
		Use:   "client [serverIP]",
		Short: "Connect to QTM server to receive mocap frames",
		Args:  cobra.ExactArgs(1),
		Run: func(_ *cobra.Command, args []string) {
			osutil.ExitIfError(
				clientLogic(osutil.CancelOnInterruptOrTerminate(nil), args[0], droneDeviceID))
		},
	}

	cmd.Flags().StringVarP(&droneDeviceID, "drone-device-id", "", droneDeviceID, "Drone ID")

	app.AddCommand(cmd)

	app.AddCommand(serverSimulatorEntrypoint())

	osutil.ExitIfError(app.Execute())
}

// the magic happens here
func clientLogic(ctx context.Context, serverIP string, droneDeviceID string) error {
	rand.Seed(time.Now().UTC().UnixNano())

	serverAddr := newQtmServerAddrWithDefaultBasePort(serverIP)

	//nolint:gosec
	localPort := int(6734 + rand.Int31n(65535-6734))

	qtmRtConn, err := net.ListenUDP("udp4", &net.UDPAddr{
		IP:   net.IPv4(0, 0, 0, 0),
		Port: localPort,
	})
	if err != nil {
		return err
	}
	// will be closed by UDP-receiver-closer

	serverAddrQTMRT, err := net.ResolveUDPAddr("udp4", serverAddr.QTMRT())
	if err != nil {
		return err
	}

	repeatNATHolepunch := time.NewTicker(9 * time.Second)
	checkTime := time.NewTicker(2 * time.Second)

	sendNATHolepunch := func() error {
		log.Println("sending NAT holepunch")
		_, err := qtmRtConn.WriteTo([]byte(""), serverAddrQTMRT) // send packet
		return err
	}

	tasks := taskrunner.New(ctx, logex.StandardLogger())

	if metricsPort := os.Getenv("METRICS_PORT"); metricsPort != "" {
		tasks.Start("metrics-server", func(ctx context.Context) error {
			return metricsHTTPServer(ctx, metricsPort)
		})
	}

	udpPacketFromQTMServer := startUDPReceiver(tasks, qtmRtConn)

	serverControlConnection, err := timeoutDialer.DialContext(ctx, "tcp", serverAddr.LittleEndian())
	if err != nil {
		return err
	}
	defer serverControlConnection.Close()

	if err := serverControlConnection.(*net.TCPConn).SetKeepAlive(true); err != nil {
		return err
	}
	if err := serverControlConnection.(*net.TCPConn).SetKeepAlivePeriod(1 * time.Second); err != nil {
		return err
	}

	if err := sendNATHolepunch(); err != nil {
		return err
	}

	bodyConf, err := clientRequestStreamingFromServer(serverControlConnection, localPort, droneDeviceID)
	if err != nil {
		return err
	}

	serverControlConnectionClosedUnexpectedly := make(chan error, 1)

	go func() {
		dummyBytes := [1500]byte{}
		_, err := serverControlConnection.Read(dummyBytes[:])
		// TODO: err can also be non-nil (but shouldn't be because server shouldn't send any more data)
		serverControlConnectionClosedUnexpectedly <- err
	}()

	lastLocationLogged := time.Time{}
	lastSuccessfulLocationUpdate := time.Now() // OK to lie so we get some time in the start to wait for the first frame
	numLogEntriesSuppressed := 0

	for {
		select {
		case err := <-tasks.Done():
			return err
		case msgBytes := <-udpPacketFromQTMServer:
			packet, err := parsePacket(bytes.NewReader(msgBytes))
			if err != nil {
				return ErrorWrap("parsePacket", err)
			}

			body, err := parseData6DOFForBody(packet, bodyConf)
			if err != nil {
				return ErrorWrap("parseData6DOFForBody", err)
			}

			suppress := time.Since(lastLocationLogged) < 2*time.Second

			if !suppress {
				if body.HasXYZ() { // check that coords ain't NaN's
					log.Printf(
						"%s [%f, %f, %f] (+ %d after previous log msg)",
						bodyConf.lookForBodyName,
						body.X,
						body.Y,
						body.Z,
						numLogEntriesSuppressed)

					lastSuccessfulLocationUpdate = time.Now()

					locationUpdateCount.Inc()
				} else {
					log.Printf("%s not found (data is NaN's)", bodyConf.lookForBodyName)
				}

				lastLocationLogged = time.Now()
				numLogEntriesSuppressed = 0
			} else {
				numLogEntriesSuppressed++
			}
		case <-repeatNATHolepunch.C:
			if err := sendNATHolepunch(); err != nil {
				return err
			}
		case <-checkTime.C:
			if time.Since(lastSuccessfulLocationUpdate) > 10*time.Second {
				return errors.New("too much time since last successful location update - stopping")
			}
		case err := <-serverControlConnectionClosedUnexpectedly:
			return fmt.Errorf("serverControlConnectionClosedUnexpectedly: %v", err)
		}
	}
}

func startUDPReceiver(tasks *taskrunner.Runner, qtmRtConn *net.UDPConn) chan []byte {
	udpPacketFromQTMServer := make(chan []byte, 1)

	tasks.Start("UDP-receiver-closer", func(ctx context.Context) error {
		<-ctx.Done()
		return qtmRtConn.Close() // to unblock UDP-receiver
	})

	tasks.Start("UDP-receiver", func(ctx context.Context) error {
		buf := [1500]byte{} // 1500 = https://serverfault.com/a/601781

		for {
			n, err := qtmRtConn.Read(buf[:])
			if err != nil {
				return IgnoreErrorIfCanceled(ctx, err)
			}

			msg := make([]byte, n)
			copy(msg, buf[:n])

			select {
			case <-ctx.Done():
				return nil
			case udpPacketFromQTMServer <- msg:
				// no-op
			}
		}
	})

	return udpPacketFromQTMServer
}

var timeoutDialer = &net.Dialer{
	Timeout: 3 * time.Second,
}
