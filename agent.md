# Agent Instructions (real_scenario)

Before creating, editing, or deleting ANY file in this project, ask the
user for explicit permission first and wait for a clear yes. This applies
to every file change, no exceptions -- bug fixes, config updates,
generated/data files, anything. Do not assume a fix is "obviously fine to
just do." Describe the change you want to make and why, then wait.

If a file looks missing or stale, do not regenerate/replace it on your own
assumption -- check `git log`/`git diff`/`git status` for that exact path
first, since it may already exist upstream or have history you have not
seen. Confirm with the user before writing a replacement.

## Why this rule exists

On 2026-08-26, Claude assumed `Parcel_Sorting_Map.yaml`/`.png` (the AMR's
navigation map config/image) were missing and generated fabricated
placeholder versions to hand to a teammate building a control/monitoring
system, without checking git history first. Both files were already
committed with real values (`origin: [-17.975, -11.975, 0.0000]`) and got
silently overwritten. The user had to catch this and have it reverted.

## Scope

This is a hard rule for this project (`/home/rokey/real_scenario`) that
overrides any general "just fix it" instinct. When in doubt, ask.
