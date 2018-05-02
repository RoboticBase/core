FROM mhart/alpine-node:8

MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV GIT_URL_IOTA "https://github.com/telefonicaid/iotagent-ul.git"
ENV IOTA_HOME "/opt/iotaul"

ARG GIT_REV_IOTA="master"

RUN echo "INFO: Cloning IoTagent-ul from ${GIT_URL_IOTA}:${GIT_REV_IOTA}" && \
   apk add --update git && \
   git clone ${GIT_URL_IOTA} ${IOTA_HOME} && \
   cd ${IOTA_HOME} && \
   git checkout ${GIT_REV_IOTA} && \
   npm install && \
   rm ./config.js

WORKDIR ${IOTA_HOME}
ADD ./config.js ./

EXPOSE 4041
EXPOSE 7896
ENTRYPOINT ["bin/iotagent-ul", "config.js"]
