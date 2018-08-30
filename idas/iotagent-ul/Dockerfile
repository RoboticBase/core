# Copyright 2015 Telefónica Investigación y Desarrollo, S.A.U
#
# This file is part of the IoT Agent for the Ultralight 2.0 protocol (IOTAUL) component
#
# IOTAUL is free software: you can redistribute it and/or
# modify it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of the License,
# or (at your option) any later version.
#
# IOTAUL is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public
# License along with IOTAUL.
# If not, see http://www.gnu.org/licenses/.
#
# For those usages not covered by the GNU Affero General Public License
# please contact with: [daniel.moranjimenez@telefonica.com]

# modified Nobuyuki to use node:8.11-alpine by Matsui<nobiyuki.matsui@gmail.com>

FROM node:8.11-alpine

MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

RUN apk update && \
    apk add --no-cache --virtual .build git && \
    git clone https://github.com/telefonicaid/iotagent-ul.git /opt/iotagent-ul && \
    cd /opt/iotagent-ul && \
    git checkout 290a1fac38e6964e5a7f1ee97a3c611a111df418 && \
    rm -f ./config.js && ln -s /etc/iotagent-ul/config.js && \
    npm install --production && \
    apk del --purge .build

WORKDIR /opt/iotagent-ul

EXPOSE 4041
EXPOSE 7896
ENTRYPOINT ["bin/iotagent-ul", "config.js"]
