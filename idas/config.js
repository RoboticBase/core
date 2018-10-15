/*
 * Copyright 2015 Telefonica Investigación y Desarrollo, S.A.U
 *
 * This file is part of iotagent-mqtt
 *
 * iotagent-mqtt is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * iotagent-mqtt is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with iotagent-mqtt.
 * If not, seehttp://www.gnu.org/licenses/.
 *
 * For those usages not covered by the GNU Affero General Public License
 * please contact with::[contacto@tid.es]
 */
var config = {};

config.mqtt = {};

config.amqp = {
    host: "rabbitmq-amqp",
    port: 5672,
    username: "iotagent",
    password: "${IOTA_PASSWORD}",
    exchange: "iota-exchange",
    queue: "iotaqueue",
    options: {durable: true}
};

config.http = {};

config.iota = {
    logLevel: "INFO",
    timestamp: true,
    contextBroker: {
        host: "orion",
        port: '1026'
    },
    server: {
        port: 4041
    },
    deviceRegistry: {
        type: "mongodb"
    },
    mongodb: {
        host: "mongodb-0.mongodb, mongodb-1.mongodb, mongodb-2.mongodb",
        port: "27017",
        db: "iotagentul",
        replicaSet: "rs0"
    },
    types: {},
    service: "roboticsbase",
    subservice: "/",
    providerUrl: "http://iotagent-ul:4041",
    deviceRegistrationDuration: "P1M",
    defaultType: "fiware",
    defaultResource: "/iot/d"
};

config.defaultKey = 'TEF';
config.defaultBinding = 'AMQP';

module.exports = config;
