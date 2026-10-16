# Running GUI API

The opt-in GUI API lets an agent inspect the running Orca instance and request its
normal preset synchronization without clicking the UI. It requires a rebuilt GUI
containing the API implementation. Installing the Python client alone does not add
an API to an existing packaged GUI. The API provides status, preset snapshots and
preset synchronization; it has no print-submission actions.

## Start and authenticate

Start the rebuilt GUI with `ORCA_AGENT_API_TOKEN` set to a private random secret of
at least 32 characters. The listener defaults to `127.0.0.1:13619`;
`ORCA_AGENT_API_PORT` changes its port. Keep the token private and out of command
arguments, logs and version control. This is a local API secret, separate from Orca
cloud credentials. Cloud authentication remains owned by the running GUI.

The standard-library Python client accepts `--token-file` or, when no file is
specified, reads `ORCA_AGENT_API_TOKEN`. Use printable ASCII without whitespace for
the token; a token file may end with a newline. For example, after starting the GUI
with the same secret:

```sh
python scripts/agent_gui.py status --token-file /private/path/gui-api-token
python scripts/agent_gui.py presets --token-file /private/path/gui-api-token
python scripts/agent_gui.py sync --wait --timeout 120 --token-file /private/path/gui-api-token
```

The client accepts only HTTP loopback IP origins, bypasses HTTP proxies and never
follows redirects. For a GUI on another computer, forward its loopback port through
SSH and use the local end of the tunnel:

```sh
ssh -N -L 13620:127.0.0.1:13619 gui-host
python scripts/agent_gui.py status --url http://127.0.0.1:13620 --token-file /private/path/gui-api-token
```

## Responses and completion

Commands write one JSON object to stdout. `status` and `presets` wrap the API snapshot
in `{"status":"ok","result":...}`. Presets expose saved and active snapshots: an
active edited configuration need not equal its saved preset. Status reports readiness,
login/stealth state and the latest synchronization job.

Snapshots include names, cloud IDs/revisions, synchronization flags, dirty key names
and separate saved/edited `config_sha256` fingerprints. Fingerprints hash compact,
sorted-key UTF-8 JSON mapping native configuration keys to their `opt_serialize`
strings. Configuration values and printer credentials are not returned. The native
serialization format matters when comparing these hashes with external snapshots.

`sync` sends an empty POST body. HTTP 202 yields `status: "accepted"` and a `job_id`;
this proves acceptance only. With `--wait`, the client checks status at one-second
intervals and reports `applied` only when that same job reaches `applied` after the
GUI reload completes. Intermediate states are `accepted`, `waiting`, `pulling` and
`reloading`; `failed` and `cancelled` become client errors. A changed job ID is a
conflict, never evidence that the requested job succeeded.
Job IDs are random per request and remain distinct across GUI restarts. Unresolved
local/cloud conflicts fail the job even when the GUI reload itself completed.

| Client status | Exit code | Meaning |
| --- | --- | --- |
| `ok` | 0 | Read-only snapshot received. |
| `accepted` | 0 | Sync queued; application is not established. |
| `applied` | 0 | Matching job completed GUI reload. |
| `error` | 1 | Authentication, transport, protocol or job failure. |
| `conflict` | 3 | HTTP 409 or the awaited job was replaced. |
| `timeout` | 4 | Waiting expired; accepted work may still finish. |

The timeout defaults to 60 seconds and permits values above zero through 600.
Individual HTTP operations are capped at ten seconds and the remaining wait budget.
The client never automatically retries a sync POST. If a request times out before
its response arrives, acceptance may be unknown; inspect `status` before requesting
another sync. After acceptance, polling failures retain the accepted job ID in output.
HTTP 401 indicates local API authentication failure; 409 indicates busy/conflicting
GUI state; 503 indicates the GUI is unavailable for synchronization, logged out or
in stealth mode. Raw HTTP error bodies and redirect destinations are not printed.

Synchronization uses the normal GUI pipeline and preserves dirty configurations;
it does not deliberately save or discard project edits or force cloud conflicts.
An `applied` job is not proof that an active dirty configuration now equals a cloud
preset. Inspect the active/saved snapshots after completion. A busy GUI can reject
the request; the client does not bypass GUI state checks.
Completion covers the initial cloud pull and GUI reload. The normal background
upload/subscription loop continues independently.

## HTTP contract

Every request uses `Authorization: Bearer <local-secret>`.

| Method and path | Response |
| --- | --- |
| `GET /v1/status` | HTTP 200, GUI status and `sync` object. |
| `GET /v1/presets` | HTTP 200, saved and active preset snapshots. |
| `POST /v1/presets/sync` | Empty body; HTTP 202, status containing accepted `sync.job_id`. |

The `sync` object contains `job_id`, `state`, `busy` and `error`. Before a job starts,
its state is `idle`. The endpoint reports errors using an appropriate HTTP status;
the client distinguishes a successful read from accepted or applied mutation.

## Verification

```sh
python tests/cli/test_agent_gui.py
```

These mock-loopback tests cover authentication, token-file precedence, secret
redaction, asynchronous completion, job correlation, failure/cancellation, timeout,
HTTP conflicts, redirect rejection and URL validation. They do not establish that
a particular GUI binary has the API or that a live cloud sync preserves a particular
open project; those require running-GUI integration checks.
