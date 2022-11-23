package main

// QTM server client

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"log"
	"net"
	"strconv"

	. "github.com/function61/gokit/builtin"
	"github.com/go-restruct/restruct"
)

// encapsulates logic for the different port numbers:
//
//	https://docs.qualisys.com/qtm-rt-protocol/#ip-port-numbers
type qtmServerAddr struct {
	ip       string
	basePort int
}

// the default base port is 22222
func newQtmServerAddrWithDefaultBasePort(ip string) qtmServerAddr {
	return qtmServerAddr{
		ip:       ip,
		basePort: 22222,
	}
}

// Little-endian version of the protocol. Used from protocol version 1.1 and onwards.
func (q qtmServerAddr) LittleEndian() string {
	return net.JoinHostPort(q.ip, strconv.Itoa(q.basePort+1))
}

// QTM RT-protocol over OSC (Open Sound Control) protocol. OSC protocol is sent over UDP.
func (q qtmServerAddr) QTMRT() string {
	return net.JoinHostPort(q.ip, strconv.Itoa(q.basePort+3))
}

func clientRequestStreamingFromServer(server io.ReadWriter, localPort int, droneDeviceID string) (*bodyConfiguration, error) {
	if err := clientReceiveGreetingAndNegotiateProtocolVersion(server); err != nil {
		return nil, ErrorWrap("clientReceiveGreetingAndNegotiateProtocolVersion", err)
	}

	parametersResp, err := clientExchangePackets(server, createCommandPacket("GetParameters 6D"))
	if err != nil {
		return nil, err
	}

	parametersPayload, err := parametersResp.GetPayload(PacketTypeXML)
	if err != nil {
		return nil, err
	}

	bodyConf, err := parseBodyConfiguration(parametersPayload, droneDeviceID)
	if err != nil {
		return nil, err
	}

	if localPort == 0 { // use TCP
		// server will actually stream lots of data packets, but this exchange packets will assume one
		firstFrame, err := clientExchangePackets(server, createCommandPacket("StreamFrames AllFrames 6D ")) // <-- space SIC
		if err != nil {
			return nil, err
		}

		body, err := parseData6DOFForBody(firstFrame, bodyConf)
		if err != nil {
			return nil, err
		}

		if body.HasXYZ() {
			log.Printf("body=%v", body)
		} else {
			log.Printf("%s coordinates are full of NaN's", droneDeviceID)
		}
	} else {
		// server will not give a response to this. instead the control connection will be kept open.
		if _, err := server.Write(createCommandPacket(fmt.Sprintf("StreamFrames AllFrames UDP:%d 6D ", localPort)).Serialize()); err != nil {
			return nil, err
		}
	}

	return bodyConf, nil
}

func parseData6DOFForBody(frame *PacketWithPayload, bodyConf *bodyConfiguration) (*ComponentData6DOF, error) {
	data, err := frame.ParseData()
	if err != nil {
		return nil, err
	}

	for _, component := range data.Components {
		if component.Type != ComponentType6D {
			return nil, fmt.Errorf("unsupported component type: %s", component.Type.String())
		}

		componentData := ComponentData6DOFHeader{}
		if err := restruct.Unpack(component.Data, binary.LittleEndian, &componentData); err != nil {
			return nil, err
		}

		// validate assumptions
		if len(componentData.Bodies) != bodyConf.BodyCount() {
			return nil, fmt.Errorf(
				"mismatch in server-sent body count (%d) vs config (%d)",
				len(componentData.Bodies),
				bodyConf.BodyCount())
		}

		//nolint: staticcheck
		return &componentData.Bodies[bodyConf.lookForBodyIdx], nil
	}

	return nil, fmt.Errorf("body not found in data: %s", bodyConf.lookForBodyName)
}

func clientReceiveGreetingAndNegotiateProtocolVersion(conn io.ReadWriter) error {
	helloMsg, err := clientExchangePackets(conn, nil)
	if err != nil {
		return err
	}

	hello, err := helloMsg.GetCommandString()
	if err != nil {
		return err
	}

	if hello != "QTM RT Interface connected" {
		return fmt.Errorf("invalid hello magic: %s", hello)
	}

	versionConfirmationMsg, err := clientExchangePackets(conn, createCommandPacket("Version 1.19"))
	if err != nil {
		return err
	}

	versionConfirmation, err := versionConfirmationMsg.GetCommandString()
	if err != nil {
		return err
	}

	if versionConfirmation != "Version set to 1.19" {
		return fmt.Errorf("bad confirmation: %s", versionConfirmation)
	}

	stateResp, err := clientExchangePackets(conn, createCommandPacket("GetState"))
	if err != nil {
		return err
	}
	statePayload, err := stateResp.GetPayload(PacketTypeEvent)
	if err != nil {
		return err
	}

	if !bytes.Equal(statePayload, []byte{0x01}) { // TODO: what does this mean?
		return fmt.Errorf("unexpected state: %x", statePayload)
	}

	return nil
}

func clientExchangePackets(server io.ReadWriter, req *PacketWithPayload) (*PacketWithPayload, error) {
	if req != nil {
		if _, err := server.Write(req.Serialize()); err != nil {
			return nil, fmt.Errorf("clientExchangePackets: write: %v", err)
		}
	}

	response, err := parsePacket(server)
	if err != nil {
		return nil, fmt.Errorf("clientExchangePackets: receive: %v", err)
	}

	return response, nil
}
