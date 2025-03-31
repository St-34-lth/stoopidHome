#!/bin/bash
set -e

# Server

# Generate a server key.

openssl genrsa -aes256 -out server.key 2048

# Generate a server key without encryption.

openssl genrsa -out server.key 2048

# Generate a certificate signing request to send to the CA.

openssl req -out server.csr -key server.key -new

openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt -days 3060
