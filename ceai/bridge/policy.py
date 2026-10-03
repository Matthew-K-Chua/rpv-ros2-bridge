"""The HTTP round-trip to the VLFM policy server."""
import requests


class PolicyClient:

    def __init__(self, url, timeout, logger=None):
        self.url = url
        self.timeout = float(timeout)
        self._logger = logger

    def post(self, payload):
        """The server's JSON reply, or None if the step could not be taken."""
        try:
            response = requests.post(self.url, json=payload, timeout=self.timeout)
        except Exception as exc:
            self._error(f"policy POST failed: {exc}")
            return None
        if response.status_code >= 400:
            self._report_error_body(response)
            return None
        return response.json()

    def _report_error_body(self, response):
        """The server packs the real cause and traceback into the JSON body;
        surface that instead of a bare '500 Server Error'."""
        try:
            body = response.json()
        except ValueError:
            self._error(f"policy server {response.status_code}: {response.text[:2000]}")
            return
        self._error(f"policy server {response.status_code} "
                    f"{body.get('type', '')}: {body.get('error', '')}\n"
                    f"{body.get('traceback', '')}")

    def _error(self, message):
        if self._logger:
            self._logger.error(message)
