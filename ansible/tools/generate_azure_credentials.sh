#!/bin/bash
set -o nounset -o errexit -o pipefail

# login azure
az login

# get subscription id
SUBSCRIPTION_ID=$(az account show --query [id] --output tsv)

# create service principal
SERVICE_PRINCIPAL=$(az ad sp create-for-rbac --query [appId,password,tenant])
APP_ID=$(jq -r ".[0]" <<< "${SERVICE_PRINCIPAL}")
APP_PASSWORD=$(jq -r ".[1]" <<< "${SERVICE_PRINCIPAL}")
TENANT=$(jq -r ".[2]" <<< "${SERVICE_PRINCIPAL}")

cat << EOF > ~/.azure/credentials
[default]
subscription_id=${SUBSCRIPTION_ID}
client_id=${APP_ID}
secret=${APP_PASSWORD}
tenant=${TENANT}
EOF
