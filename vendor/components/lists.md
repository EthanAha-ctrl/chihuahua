# Component sources

| Category | Component / materials | Local files | Status |
| --- | --- | --- | --- |
| Motor | CubeMars RI20 / RI30 original STEP, drawings and source snapshots | [motor/](motor/), [provenance and SHA-256](motor/sources.json), [notes](motor/README.md) | Historical tendon assembly study; not a finalized selection |
| Joint | Legacy actuator catalog and default joint assignments | [joint/lists.md](joint/lists.md) | Placeholder parameters; not verified product specifications |
| Battery | Battery and electronics catalog, mounting and placement | [battery/lists.md](battery/lists.md) | Legacy placeholder parameters |
| Material | Material properties and defaults | [material/lists.md](material/lists.md) | Conservative estimates; not calibrated coupon data |
| AI board | Compute boards | [ai_board/](ai_board/) | No vendor files yet |
| Tendon | Purchased tendon components | [tendon/](tendon/) | No vendor files yet |

Current actuator candidates and supplier links: [README](../../README.md#actuator-vendor-list).
Custom tendon CAD code: `src/components/tendon/`; design notes: [RI study](../../docs/ri_tendon_module.md).
