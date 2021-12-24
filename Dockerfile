FROM alpine:3.12
RUN echo "@community http://dl-cdn.alpinelinux.org/alpine/v3.12/community" >> /etc/apk/repositories
RUN apk add --no-cache build-base libffi-dev openssl-dev python3-dev curl krb5-dev linux-headers zeromq-dev lapack-dev blas-dev redis
RUN apk add cmake gcc libxml2 automake g++ subversion libxml2-dev libxslt-dev gfortran jpeg-dev py3-pip
RUN apk add tmux vim mercurial iw
RUN apk add bash # required by codecov GH action
RUN apk add raspberrypi; exit 0 # Only succeeds on raspberry pi but not needed otherwise.
RUN apk add py3-scipy py3-numpy-dev
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install wheel certifi==2020.06.20 pytest pytest-mock coverage[toml]
COPY server/requirements.txt /install/server/
RUN python3 -m pip install -r /install/server/requirements.txt
COPY vehicle/requirements.txt /install/vehicle/
RUN python3 -m pip install -r /install/vehicle/requirements.txt
RUN python3 -m pip install pytest-watch
