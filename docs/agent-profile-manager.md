# Agent profile manager

`python scripts/agent_profiles.py` manages process, filament and single-nozzle machine
families outside Orca's inheritance graph. Each exported preset is a standalone user
root with empty `inherits` and `base_id`. The family file holds shared settings and
explicit per-nozzle overrides. An override equal to the base remains pinned when the
base changes. Commands return JSON. Completed commands exit 0; invocation errors exit 2.
A completed command can report a `pending`, `conflicted` or `failed` operation: agents
must inspect `result.status`, not just the exit code.

The script uses Python's standard library. It is an external tool in this repository,
not an option built into the Orca executable. It does not start prints.

## Import and edit

Start from standalone user preset JSON files. Input presets with parents or includes
are rejected: removing those links without resolving their settings would lose values.
Variants must contain every setting present in the base. Export equally resolved
presets before grouping a mixture of sparse and full configurations.

```sh
python scripts/agent_profiles.py family create --name 'Repro Normal' \
  --base normal-04.json --base-nozzle 0.4 \
  --variant 0.6=normal-06.json --out normal.family.json

python scripts/agent_profiles.py family show normal.family.json
python scripts/agent_profiles.py family set normal.family.json \
  --expect REVISION_FROM_SHOW --key default_acceleration --value '["5000","5000"]'
python scripts/agent_profiles.py family set normal.family.json \
  --expect NEW_REVISION --nozzle 0.6 --key layer_height --value '"0.24"'
python scripts/agent_profiles.py family reset normal.family.json \
  --expect NEW_REVISION --nozzle 0.6 --key layer_height
```

Without `--nozzle`, set changes the shared base. With it, set pins that variant's
value; reset removes the override. Names, compatibility, settings and nozzle variants
come from the imported profiles. The manager does not guess calibration values or
scale temperatures, flow limits, retraction or line widths. Machine exports set the
single nozzle diameter from the family's variant. Use separate families for different
printers and preset types. Multi-extruder machine-family creation is not supported.

Nozzle sizes and variant names are established at creation. Rebuilding a family
from different inputs creates new manager IDs; retain the original family file to
retain its IDs. Cloud IDs are explicitly selected from a cloud snapshot when pushing.

## Inspect, export and verify

```sh
python scripts/agent_profiles.py family build normal.family.json --out exported
python scripts/agent_profiles.py profiles list exported
python scripts/agent_profiles.py profiles show 'exported/process/base/Normal 0.4.json'
python scripts/agent_profiles.py profiles compare old.json new.json
python scripts/agent_profiles.py profiles verify new.json \
  --orca-command '["/path/to/orca-slicer"]'
```

Build creates a new directory with ordinary `TYPE/base/NAME.json` files and a manifest.
It does not install or publish them. Filenames match preset names because Orca's
directory loader uses filenames as identities. Names must be portable across Windows,
Linux and macOS. Use Orca's import UI to import exported presets into the active account.

`validate` and `verify` both run the supplied Orca executable in a fresh temporary data
directory, load the profile, export its effective settings, and compare the provided
settings with readback. Unknown, dropped and changed settings fail. This verifies
configuration round trips, not print quality or every printer/material combination.
Compatibility metadata is retained but is absent from project-settings export and
therefore is not covered by the value comparison. Retain explicit compatible-printer
lists appropriate to each nozzle. Defaults omitted from a root preset still come from
the installed Orca version.

`--orca-command` is a JSON argument array, allowing launch wrappers without shell
evaluation. For example `["/path/to/orca-slicer-env","/path/to/orca-slicer"]`.
Use the intended installed version when validating profiles.

## Preview, apply and rollback

```sh
python scripts/agent_profiles.py profiles plan normal.family.json \
  --library ./managed-library --out plan.json
python scripts/agent_profiles.py profiles apply plan.json \
  --orca-command '["/path/to/orca-slicer"]'
python scripts/agent_profiles.py profiles rollback PATH_TO_RECEIPT \
  --orca-command '["/path/to/orca-slicer"]'
```

Plan records each file's previous content and proposed content. Apply checks all files
against that snapshot and verifies every resulting profile before writing. A stale
plan fails. Each write uses atomic replacement, and an ordinary write failure restores
earlier files. A journal under `.agent-profile-history` retains before/after values.
Rollback also checks for intervening edits. A whole multi-file update is not atomic
across power loss; the pending journal is retained for recovery. Unfinished journals
block subsequent apply operations until recovered:

```sh
python scripts/agent_profiles.py profiles recover PATH_TO_RECEIPT \
  --direction finish --orca-command '["/path/to/orca-slicer"]'
# Or use --direction rollback to restore the journal's original files.
```

Recovery accepts only files matching either the recorded before or after version.
An intervening edit is preserved and reported as a conflict. Locks are OS-owned and
release when a process exits; leave their lock files in place. A legacy directory
lock is not removed automatically. Local recovery is file-level, not a three-way merge.

Use a dedicated managed library. The OS lock coordinates manager processes,
not Orca GUI processes or other applications. Writes beneath directories named
`OrcaSlicer`, or containing `OrcaSlicer.conf`/`OrcaSlicer.ini`, are blocked. Symlinked
managed paths are rejected. This catches conventional and marked custom data stores;
it cannot recognize an arbitrary unmarked GUI directory. Never use an active GUI
account directory as a managed library. There is no live GUI collection refresh or
unsaved-edit inspection. Concurrent external editors must also be stopped during apply.
Local apply reports `cloud_synced: false` explicitly.

## Cloud synchronization

Cloud operations require an access token file obtained through Orca authentication
and the expected account UUID. The tool checks the token's account identity; the
server verifies authentication. Token acquisition and reading the GUI's encrypted
credential store are delegated to your credential provider. An optional
`--refresh-command '["/path/to/provider", "--refresh"]'` integrates that provider:
on HTTP 401, the manager takes a token-file refresh lock, runs the argv without a
shell, and retries once. The provider receives `ORCA_PROFILE_TOKEN_FILE` and
`ORCA_PROFILE_ACCOUNT` through its environment, must atomically replace the access
token file, and must exit within 45 seconds. Its stdout/stderr are suppressed.
Credentials from a different account are rejected. No refresh token is stored in
operation journals. Providers that share encrypted refresh-token stores must also
coordinate access to that store with Orca; the manager's lock covers its token file only.
Keep token files private and out of repositories. The manager never prints their contents.

```sh
python scripts/agent_profiles.py cloud pull --token-file TOKEN_FILE \
  --account ACCOUNT_UUID --out cloud-snapshot.json
python scripts/agent_profiles.py cloud push --token-file TOKEN_FILE \
  --account ACCOUNT_UUID --snapshot cloud-snapshot.json --id EXISTING_PROFILE_UUID \
  --profile standalone-profile.json --out cloud-receipt.json \
  --orca-command '["/path/to/orca-slicer"]'
```

Use the existing cloud ID to update a profile in place. A fresh UUID creates a new
profile; the server may reject a duplicate or previously deleted identity. Name
collisions with another live ID are rejected locally. Push validates both the local
format and cloud serialization through Orca, rechecks the server revision, submits
`original_updated_time`, and compares a fresh cloud readback with the exact submitted
content. It never force-pushes. Both Standard and High Flow vector entries are retained.

A durable receipt is written **before** the request. It records the original cloud
revision and content, desired content, attempt count and operation state:

| State | Meaning / next action |
| --- | --- |
| `pending` | Outcome unknown or readback delayed; resume this receipt. |
| `verified` | Fresh cloud content matches the requested name and settings. |
| `conflicted` | Remote edit, deletion or name collision; inspect and make a new plan. |
| `failed` | Explicit rejection/authentication failure; inspect the reason. |

```sh
python scripts/agent_profiles.py cloud status cloud-receipt.json
python scripts/agent_profiles.py cloud resume cloud-receipt.json \
  --token-file TOKEN_FILE --account ACCOUNT_UUID --polls 3 --interval 1
```

Resume reconciles before sending anything. An uncertain previous upload is **not**
replayed by default. `--retry` permits replay only after the original baseline is
observed, preserving the UUID and original revision; at most three submissions are
allowed per receipt. Authentication failures can also be resumed with `--retry`
after credentials are repaired. Other rejected/conflicted operations need a new plan.
`--polls` allows 1–20 polling rounds, with `--interval` of 0–10 seconds between them.
A submission gets one additional immediate readback. Each HTTP request times out after
15 seconds; authentication refresh can add time. A matching result is idempotent and
requires no upload. Content is checked as well as timestamp, including same-second edits.

Receipts contain profile settings and previous versions, not credentials. Keep them
private. Cloud edits are one profile at a time; there is no multi-profile cloud
transaction or automatic cloud rollback. To restore an old version, push its content
against a fresh snapshot and the same ID. Server revision checking remains essential:
preflight reads cannot themselves exclude another cloud writer.

## Three-way local/cloud reconciliation

Bind an initially matching managed file and cloud UUID, then retain that binding as
the common base. Use a fresh pull for initial binding. Binding does not overwrite either
side; mismatched initial versions require explicit reconciliation first.

```sh
python scripts/agent_profiles.py sync bind \
  --profile 'managed-library/process/base/Normal 0.4.json' \
  --snapshot cloud-snapshot.json --account ACCOUNT_UUID --id PROFILE_UUID \
  --out normal.binding.json
python scripts/agent_profiles.py sync plan normal.binding.json \
  --token-file TOKEN_FILE --account ACCOUNT_UUID --out sync-plan.json
python scripts/agent_profiles.py sync apply sync-plan.json \
  --token-file TOKEN_FILE --account ACCOUNT_UUID --out sync-operation.json \
  --orca-command '["/path/to/orca-slicer"]'
python scripts/agent_profiles.py sync resume sync-operation.json \
  --token-file TOKEN_FILE --account ACCOUNT_UUID \
  --orca-command '["/path/to/orca-slicer"]'
python scripts/agent_profiles.py sync status sync-operation.json
```

Planning compares base, local file and fresh cloud content per setting. Independent
changes and deletions merge; different edits to the same setting report all three
values without choosing a winner. Comparisons normalize local arrays to cloud strings;
otherwise they are deliberately exact, so formatting-only differences can require
review. Profile renames/type changes require explicit migration and rebinding.
Older cloud revisions yield a pending plan, which cannot be applied.

Apply rechecks the binding and local revision, validates the merged profile through
Orca, publishes with cloud revision protection, verifies cloud readback, and only then
writes local content and advances the common base. A durable library reservation
blocks competing manager writes while an outcome is pending. Resume recognizes a
crash after cloud publication, after local replacement, or after baseline replacement.
If a later local edit conflicts, it is preserved; already committed cloud changes are
not silently rolled back. Inspect that receipt and create a fresh three-way plan.
Retain bindings, journals and reservation files together; do not edit/delete them to
bypass unresolved operations. Back up the managed workspace.

All cloud/sync operation results report `gui_loaded: "unknown"`. `verified` proves the
observed cloud value (and the managed file for sync), **not** that Orca's open GUI has
loaded it or that another client cannot subsequently replace it. In the GUI, save or
discard unsaved edits deliberately, then use **File → Sync Presets** and inspect the
loaded preset. This tool does not offer unattended synchronization of GUI memory.

## Tests

```sh
python tests/cli/test_agent_profiles.py
ORCA_PROFILE_COMMAND='["/path/to/orca-slicer"]' python tests/cli/test_agent_profiles.py
```

The optional real-binary check tests both nozzle variants in local and cloud formats
and rejection of an unknown key. Other tests cover pinned equal overrides, shared
edits, reset, identity stability, stale plans, rollback, partial write failure,
account targeting, same-revision conflicts, timeout-after-commit, delayed readback,
bounded replay, authentication refresh/failure, three-way changes/deletions, OS lock
release after process exit, and interrupted local/cloud commit recovery. The real
binary check runs on Linux here; Windows locking has a platform-specific implementation
but requires Windows runtime validation. Tests use temporary libraries and mocked
cloud transport; they do not modify live user profiles.
