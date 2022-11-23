package main

import (
	"encoding/xml"
	"fmt"
)

// file format of the XML parameters
type QTMParameters struct {
	XMLName xml.Name           `xml:"QTM_Parameters_Ver_1.19"`
	Bodies  []QTMParameterBody `xml:"The_6D>Body"`
}

type QTMParameterBody struct {
	Name   string                  `xml:"Name"`
	Points []QTMParameterBodyPoint `xml:"Point"`
}

type QTMParameterBodyPoint struct {
	X          float32 `xml:"X"`
	Y          float32 `xml:"Y"`
	Z          float32 `xml:"Z"`
	Virtual    bool    `xml:"Virtual"`
	PhysicalId int     `xml:"PhysicalId"`
}

type bodyConfiguration struct {
	params          QTMParameters
	lookForBodyName string
	lookForBodyIdx  int
}

func parseBodyConfiguration(qtmParametersXML []byte, droneDeviceID string) (*bodyConfiguration, error) {
	qtmParameters := QTMParameters{}
	if err := xml.Unmarshal(qtmParametersXML, &qtmParameters); err != nil {
		return nil, err
	}

	if len(qtmParameters.Bodies) == 0 {
		return nil, fmt.Errorf("invalid QTMParameters XML (or no bodies found): %s", string(qtmParametersXML))
	}

	// translate drone name to body's index
	droneBodyIdx, err := func() (int, error) {
		for idx, body := range qtmParameters.Bodies {
			if body.Name == droneDeviceID {
				return idx, nil
			}
		}

		return -1, fmt.Errorf("body not found from QTMParameters XML: %s", droneDeviceID)
	}()
	if err != nil {
		return nil, err
	}

	return &bodyConfiguration{
		params:          qtmParameters,
		lookForBodyName: droneDeviceID,
		lookForBodyIdx:  droneBodyIdx,
	}, nil
}

func (b bodyConfiguration) BodyCount() int {
	return len(b.params.Bodies)
}
