@echo off
title Pinggy TCP Tunnel for PostgreSQL
echo ===================================================
echo Starting Pinggy TCP Tunnel (Port 5432)
echo Do not close this window while using Grafana Cloud!
echo ===================================================
echo.
ssh -p 443 -R0:127.0.0.1:5432 tcp@a.pinggy.io
pause