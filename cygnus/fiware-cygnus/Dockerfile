FROM fiware/cygnus-ngsi:latest

ENV CYGNUS_SERVICE_PORT "5050"
ENV CYGNUS_API_PORT "8081"
ENV FLUME_HOME "/opt/apache-flume"

COPY cygnus-entrypoint.sh /
COPY agent.conf ${FLUME_HOME}/conf/
RUN chmod 755 /cygnus-entrypoint.sh

# Define the entry point
ENTRYPOINT ["/cygnus-entrypoint.sh"]

# Ports used by cygnus-ngsi
EXPOSE ${CYGNUS_SERVICE_PORT} ${CYGNUS_API_PORT}
