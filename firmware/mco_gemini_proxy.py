#!/usr/bin/env python3
"""
MCO-1D Google Gemini Ground Station AI Proxy
============================================
Lightweight, zero-dependency proxy running on port 5759.
Receives telemetry and pilot queries from the Ground Station UI (mco_ai_ground_station.jsx),
routes them to Google Gemini Flash API, and returns responses compatible with the browser UI.

Usage:
    python3 firmware/mco_gemini_proxy.py
    python3 firmware/mco_gemini_proxy.py --test

Environment:
    export GEMINI_API_KEY="your-api-key"
"""

import os
import sys
import json
import argparse
import urllib.request
import urllib.error
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

PROXY_PORT = 5759
CORS_ORIGIN = "*"
DEFAULT_MODEL = "gemini-2.5-flash"

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

class GeminiProxyHandler(BaseHTTPRequestHandler):
    def _set_cors_headers(self, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", CORS_ORIGIN)
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.end_headers()

    def do_OPTIONS(self):
        self._set_cors_headers(200)

    def do_GET(self):
        if self.path in ("/health", "/"):
            self._set_cors_headers(200)
            status_data = {
                "status": "healthy",
                "service": "mco_gemini_proxy",
                "model": DEFAULT_MODEL,
                "api_key_configured": bool(os.environ.get("GEMINI_API_KEY"))
            }
            self.wfile.write(json.dumps(status_data).encode("utf-8"))
        else:
            self._set_cors_headers(404)
            self.wfile.write(b'{"error": "Not found"}')

    def do_POST(self):
        if self.path not in ("/api/anthropic", "/api/gemini"):
            self._set_cors_headers(404)
            self.wfile.write(b'{"error": "Unknown endpoint"}')
            return

        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            self._set_cors_headers(500)
            err = {"error": "GEMINI_API_KEY environment variable not set on ground station host."}
            self.wfile.write(json.dumps(err).encode("utf-8"))
            return

        content_length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_length)

        try:
            req_json = json.loads(post_data.decode("utf-8"))
        except Exception as e:
            self._set_cors_headers(400)
            self.wfile.write(json.dumps({"error": f"Invalid JSON payload: {e}"}).encode("utf-8"))
            return

        # Translate Anthropic messages format to Gemini format
        messages = req_json.get("messages", [])
        system_instruction = req_json.get("system", "")

        gemini_contents = []
        for msg in messages:
            role = "user" if msg.get("role") == "user" else "model"
            content = msg.get("content", "")
            if isinstance(content, list):
                text_parts = [part.get("text", "") for part in content if part.get("type") == "text"]
                text = "\n".join(text_parts)
            else:
                text = str(content)
            gemini_contents.append({"role": role, "parts": [{"text": text}]})

        gemini_payload = {
            "contents": gemini_contents,
            "generationConfig": {
                "maxOutputTokens": req_json.get("max_tokens", 2048),
                "temperature": 0.2
            }
        }
        if system_instruction:
            gemini_payload["systemInstruction"] = {
                "parts": [{"text": system_instruction}]
            }

        url = f"https://generativelanguage.googleapis.com/v1beta/models/{DEFAULT_MODEL}:generateContent?key={api_key}"
        
        try:
            req = urllib.request.Request(
                url,
                data=json.dumps(gemini_payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}
            )
            with urllib.request.urlopen(req, timeout=30) as resp:
                raw_resp = resp.read()
                gemini_resp = json.loads(raw_resp.decode("utf-8"))
        except urllib.error.HTTPError as e:
            err_body = e.read().decode("utf-8", errors="ignore")
            self._set_cors_headers(e.code)
            self.wfile.write(json.dumps({"error": f"Gemini API Error: {err_body}"}).encode("utf-8"))
            return
        except Exception as e:
            self._set_cors_headers(500)
            self.wfile.write(json.dumps({"error": f"Proxy request failed: {e}"}).encode("utf-8"))
            return

        # Extract generated text
        try:
            candidate = gemini_resp.get("candidates", [])[0]
            parts = candidate.get("content", {}).get("parts", [])
            reply_text = "".join([p.get("text", "") for p in parts])
        except Exception:
            reply_text = "MCO-1D Copilot: Empty response received from model."

        # Format output transparently in Anthropic JSON response schema for browser UI
        result = {
            "id": f"gemini-{os.urandom(6).hex()}",
            "type": "message",
            "role": "assistant",
            "content": [{"type": "text", "text": reply_text}],
            "model": DEFAULT_MODEL,
            "stop_reason": "end_turn",
            "usage": {
                "input_tokens": gemini_resp.get("usageMetadata", {}).get("promptTokenCount", 0),
                "output_tokens": gemini_resp.get("usageMetadata", {}).get("candidatesTokenCount", 0),
            }
        }

        self._set_cors_headers(200)
        self.wfile.write(json.dumps(result).encode("utf-8"))

def run_server(port=PROXY_PORT):
    server_address = ("", port)
    httpd = ThreadedHTTPServer(server_address, GeminiProxyHandler)
    print(f"MCO-1D Gemini AI Ground Station Proxy listening on http://localhost:{port}")
    print(f"  Forwarding to: Google Gemini API ({DEFAULT_MODEL})")
    print(f"  Ready for browser Ground Station connections.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Gemini Proxy.")
        httpd.server_close()

def run_test():
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        print("GEMINI_API_KEY not set. Cannot run live cloud test.")
        sys.exit(1)
    print("Testing Gemini Flash connection...")
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{DEFAULT_MODEL}:generateContent?key={api_key}"
    payload = {
        "contents": [{"role": "user", "parts": [{"text": "MCO-1D telemetry check: respond with 1 sentence status confirmation."}]}]
    }
    req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"), headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            text = data["candidates"][0]["content"]["parts"][0]["text"]
            print("Gemini Response:", text.strip())
            print("SUCCESS: Gemini Ground Station Proxy test passed.")
    except Exception as e:
        print("Test failed:", e)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MCO-1D Gemini Ground Station Proxy")
    parser.add_argument("--test", action="store_true", help="Test connection to Gemini API")
    parser.add_argument("--port", type=int, default=PROXY_PORT, help="Port to listen on")
    args = parser.parse_args()

    if args.test:
        run_test()
    else:
        run_server(args.port)
