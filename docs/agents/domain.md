# Domain Docs

This is a single-context repo for the RT1064 / OpenART intelligent vision project.

## Before exploring, read these

- `CONTEXT.md` at the repo root, if present.
- `docs/adr/`, if present, for architectural decisions related to the area being changed.
- `docs/competition/` for competition rules, OpenART/RT1064 division of work, UART notes, map recognition, and Push Box planning context.

If `CONTEXT.md` or `docs/adr/` does not exist, proceed silently. Do not block normal work just because those files are absent.

## Layout

Expected single-context layout:

```text
/
|-- CONTEXT.md
|-- docs/
|   |-- adr/
|   |-- agents/
|   `-- competition/
|-- libraries/
|-- openmv/
`-- project/
```

## Vocabulary

When output names project concepts, prefer the vocabulary already used in `AGENTS.md`, `docs/competition/`, and `CONTEXT.md` when present. Avoid inventing alternate names for the same hardware, toolchain, or competition task.

## ADR conflicts

If an existing ADR contradicts a proposed implementation or issue plan, surface the conflict explicitly instead of silently overriding it.
