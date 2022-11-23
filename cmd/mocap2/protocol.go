// nolint: deadcode
package main

// QTM protocol data structures, parsers etc.

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"

	"github.com/go-restruct/restruct"
)

// https://github.com/tiiuae/qualisys_cpp_sdk/blob/55ba94b913ea8a1678e4e8aa121ea5c4029eeedb/RTPacket.h#L22
type PacketType uint32

const (
	PacketTypeError      PacketType = 0
	PacketTypeCommand    PacketType = 1
	PacketTypeXML        PacketType = 2
	PacketTypeData       PacketType = 3
	PacketTypeNoMoreData PacketType = 4
	PacketTypeC3DFile    PacketType = 5
	PacketTypeEvent      PacketType = 6
	PacketTypeDiscover   PacketType = 7
	PacketTypeQTMFile    PacketType = 8
	PacketTypeNone       PacketType = 9
)

// https://docs.qualisys.com/qtm-rt-protocol/#data-component-types
type ComponentType uint32

const (
	ComponentType3D                     ComponentType = 1  // 3D marker data
	ComponentType3D_No_Labels           ComponentType = 2  // Unidentified 3D marker data
	ComponentTypeAnalog                 ComponentType = 3  // Analog data from available analog devices
	ComponentTypeForce                  ComponentType = 4  // Force data from available force plates.
	ComponentType6D                     ComponentType = 5  // 6D data - position and rotation matrix
	ComponentType6D_Euler               ComponentType = 6  // 6D data - position and Euler angles
	ComponentType2D                     ComponentType = 7  // 2D marker data
	ComponentType2D_Linearized          ComponentType = 8  // Linearized 2D marker data
	ComponentType3D_Residuals           ComponentType = 9  // 3D marker data with residuals
	ComponentType3D_No_Labels_Residuals ComponentType = 10 // Unidentified 3D marker data with residuals
	ComponentType6D_Residuals           ComponentType = 11 // 6D data - position and rotation matrix with residuals
	ComponentType6D_Euler_Residuals     ComponentType = 12 // 6D data - position and Euler angles with residuals
	ComponentTypeAnalog_Single          ComponentType = 13 // Analog data from available analog devices. Only one sample per channel and camera frame. The latest sample is used if more than one sample is available.
	ComponentTypeImage                  ComponentType = 14 // Image frame from a specific camera. Image size and format is set with the XML settings, see Image settings.
	ComponentTypeForce_Single           ComponentType = 15 // Force data from available force plates. Only one sample per plate and camera frame. The latest sample is used if more than one sample is available.
	ComponentTypeGaze_Vector            ComponentType = 16 // Gaze vector data defined by a unit vector and position.
	ComponentTypeTimecode               ComponentType = 17 // IRIG or SMPTE timecode
	ComponentTypeSkeleton               ComponentType = 18 // Skeleton segment information
	ComponentTypeEyeTracker             ComponentType = 19 // Eye tracker data with pupil size.
)

var componentTypeToString = map[ComponentType]string{
	ComponentType3D:                     "3D",
	ComponentType3D_No_Labels:           "3D_No_Labels",
	ComponentTypeAnalog:                 "Analog",
	ComponentTypeForce:                  "Force",
	ComponentType6D:                     "6D",
	ComponentType6D_Euler:               "6D_Euler",
	ComponentType2D:                     "2D",
	ComponentType2D_Linearized:          "2D_Linearized",
	ComponentType3D_Residuals:           "3D_Residuals",
	ComponentType3D_No_Labels_Residuals: "3D_No_Labels_Residuals",
	ComponentType6D_Residuals:           "6D_Residuals",
	ComponentType6D_Euler_Residuals:     "6D_Euler_Residuals",
	ComponentTypeAnalog_Single:          "Analog_Single",
	ComponentTypeImage:                  "Image",
	ComponentTypeForce_Single:           "Force_Single",
	ComponentTypeGaze_Vector:            "Gaze_Vector",
	ComponentTypeTimecode:               "Timecode",
	ComponentTypeSkeleton:               "Skeleton",
	ComponentTypeEyeTracker:             "EyeTracker",
}

func (c ComponentType) String() string {
	return componentTypeToString[c]
}

const (
	packetHeaderLen = 8 // [B]
)

type DataPayload struct {
	MarkerTimestamp   uint64 // Number of microseconds from start
	MarkerFrameNumber uint32
	ComponentCount    uint32
	Components        []Component `struct:"sizefrom=ComponentCount"`
}

type Component struct {
	Size uint32
	Type ComponentType
	Data []byte // length dynamically calculated from Size-8 in Unpack()
}

type ComponentData6DOFHeader struct {
	BodyCount       uint32
	DropRate2D      uint16              // Number of individual 2D frames that have been lost in the communication between QTM and the cameras.
	OutOfSyncRate2D uint16              // Number of individual 2D frames in the communication between QTM and the cameras, which have not had the same frame number as the other frames.
	Bodies          []ComponentData6DOF `struct:"sizefrom=BodyCount"`
}

type ComponentData6DOF struct {
	X      float32
	Y      float32
	Z      float32
	Rot3x3 [9]float32
}

func (c ComponentData6DOF) HasXYZ() bool {
	isNaN := func(num float32) bool { // helper
		return math.IsNaN(float64(num))
	}

	someUnset := isNaN(c.X) || isNaN(c.Y) || isNaN(c.Z)
	return !someUnset
}

var _ restruct.Unpacker = (*Component)(nil)

func (c *Component) Unpack(buf []byte, order binary.ByteOrder) ([]byte, error) {
	c.Size = order.Uint32(buf[0:])
	c.Type = ComponentType(order.Uint32(buf[4:]))
	c.Data = buf[8:c.Size]

	return buf[c.Size:], nil
}

// > All data sent between the server and the client is packaged in packets with an 8-byte header
// > consisting of a 4‑byte Size field and a 4‑byte Type field.
// https://docs.qualisys.com/qtm-rt-protocol/#protocol-structure
type PacketHeader struct {
	Size int32
	Type PacketType
}

type PacketWithPayload struct {
	Header  PacketHeader
	Payload []byte
}

func (p PacketWithPayload) Serialize() []byte {
	out, err := restruct.Pack(binary.LittleEndian, &p)
	if err != nil {
		panic(fmt.Errorf("restruct: %w", err))
	}

	return out
}

func (p PacketWithPayload) GetCommandString() (string, error) {
	payload, err := p.GetPayload(PacketTypeCommand)
	if err != nil {
		return "", err
	}

	if payload[len(payload)-1] != 0x00 {
		return "", fmt.Errorf("command string was not null terminated: %x", payload[len(payload)-1])
	}

	return string(p.Payload[:len(payload)-1]), nil
}

func (p PacketWithPayload) GetPayload(expectedType PacketType) ([]byte, error) {
	if p.Header.Type != expectedType {
		return nil, fmt.Errorf("packet type not command; was: %d", p.Header.Type)
	}

	return p.Payload, nil
}

func (p PacketWithPayload) ParseData() (*DataPayload, error) {
	framePayload, err := p.GetPayload(PacketTypeData)
	if err != nil {
		return nil, err
	}

	data := &DataPayload{}
	if err := restruct.Unpack(framePayload, binary.LittleEndian, data); err != nil {
		return nil, err
	}

	return data, nil
}

func parsePacket(reader io.Reader) (*PacketWithPayload, error) {
	headerBytes := [packetHeaderLen]byte{}
	if _, err := io.ReadFull(reader, headerBytes[:]); err != nil {
		return nil, err
	}

	header := PacketHeader{}
	if err := restruct.Unpack(headerBytes[:], binary.LittleEndian, &header); err != nil {
		return nil, err
	}

	payloadBuf := make([]byte, header.Size-packetHeaderLen)
	if _, err := io.ReadFull(reader, payloadBuf); err != nil {
		return nil, err
	}

	return &PacketWithPayload{
		Header:  header,
		Payload: payloadBuf,
	}, nil
}

func createCommandPacket(commandStr string) *PacketWithPayload {
	commandStrBytes := append([]byte(commandStr), 0x00) // add null termination

	return createPacket(PacketTypeCommand, commandStrBytes)
}

func createPacket(type_ PacketType, payload []byte) *PacketWithPayload {
	return &PacketWithPayload{
		Header: PacketHeader{
			Size: int32(packetHeaderLen + len(payload)),
			Type: type_,
		},
		Payload: payload,
	}
}
