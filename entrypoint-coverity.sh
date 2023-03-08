#!/bin/bash -eu

echo "running coverity scan"
export PATH=$PATH:/cov/bin/
coverity scan --exclude-language java
coverity list