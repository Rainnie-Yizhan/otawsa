@echo off
echo =========================================
echo AI IoT Gateway Startup Script
echo =========================================

:: กำหนดโดเมนคงที่ของคุณ
set MY_DOMAIN=neriah-discrepant-healthfully.ngrok-free.dev

:: เปิด ngrok
echo [1/2] Starting ngrok with Static Domain...
start cmd /k "ngrok http --domain=%MY_DOMAIN% 5678"

:: รอ 3 วินาทีให้ ngrok ตั้งหลัก
timeout /t 3 /nobreak > NUL

:: ตั้งค่า Webhook และเปิด n8n
echo [2/2] Starting n8n...
set WEBHOOK_URL=https://%MY_DOMAIN%
start cmd /k "n8n start"

echo =========================================
echo System is running! You can close this window.
echo =========================================
