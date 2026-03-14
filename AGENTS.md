# Documentation


## CTRE Phoenix 6
Reference doc for CTRE Phoenix 6 Status Signals:
https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html

### Status Signal Refresh Behavior
When calling `motor.getSignal(boolean waitForFresh)`:
- **`waitForFresh = true`** (default): Waits for the next scheduled signal update from the device before returning. This ensures fresh data but introduces latency.
- **`waitForFresh = false`**: Returns immediately with the last known value without waiting. Recommended for subsystem periodic loops where signals are refreshed via `BaseStatusSignal.refreshAll()`.

### Batch Refresh with BaseStatusSignal.refreshAll()
Use `BaseStatusSignal.refreshAll(statusSignal1, statusSignal2, ...)` to refresh multiple signals in **parallel** with a single CAN transaction, rather than fetching them individually. This is significantly more efficient and is the preferred pattern for periodic updates.

**Example with TalonFXState:**
```java
// In subsystem periodic
TalonFXState.refreshAll(extendLeftState, extendRightState);
// Now all signals in both states have been refreshed in parallel
```


