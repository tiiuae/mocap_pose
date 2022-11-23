package main

import (
	"testing"

	"github.com/function61/gokit/testing/assert"
)

func TestMain(t *testing.T) {
	_, err := clientRequestStreamingFromServer(&qtmServerSimulator{}, 0, "sad033")
	assert.EqualString(t, err.Error(), "body not found from QTMParameters XML: sad033")

	_, err = clientRequestStreamingFromServer(&qtmServerSimulator{}, 0, "sad04")
	assert.Ok(t, err)
}
