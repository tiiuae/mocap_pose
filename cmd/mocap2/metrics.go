package main

// Prometheus metrics so we can observe whether this program works properly.

import (
	"context"
	"net/http"

	"github.com/function61/gokit/net/http/httputils"
	"github.com/prometheus/client_golang/prometheus"
	"github.com/prometheus/client_golang/prometheus/promhttp"
)

var (
	locationUpdateCount = prometheus.NewCounter(
		prometheus.CounterOpts{
			Name: "location_update_count",
			Help: "Number of location updates received from Mocap server",
		},
	)
)

func init() {
	prometheus.MustRegister(locationUpdateCount)
}

func metricsHTTPServer(ctx context.Context, metricsPort string) error {
	http.Handle("/metrics", promhttp.Handler())

	//nolint: gosec
	srv := &http.Server{
		Addr: ":" + metricsPort,
	}

	return httputils.CancelableServer(ctx, srv, srv.ListenAndServe)
}
