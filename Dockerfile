FROM alpine:latest

CMD ["/usr/bin/mocap2", "client", "172.18.32.20"]

ADD rel/mocap2_linux-amd64 /usr/bin/mocap2
