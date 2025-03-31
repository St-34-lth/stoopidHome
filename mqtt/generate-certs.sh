#!/bin/bash
set -e

# CA

# Generate a certificate authority certificate and key.

openssl req -new -x509 -days 3060 -extensions v3_ca -keyout ca.key -out ca.crt
