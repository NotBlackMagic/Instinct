# Parameter Service (`ParamService`)
The Instinct firmware uses a decentralized, zero-allocation runtime parameter registry (`Instinct/Modules/System/Parameters/paramService.hpp`). Instead of maintaining a single monolithic parameter table, each module owns its own static variables and registers a `constexpr` descriptor array at boot.

Because `ParamService` binds each `ParamEntry` directly to a module's local `float` or `int32_t` variable via raw pointers, reading a parameter is just like a normal variable read with zero lookup overhead. All `ParamList` nodes and `ParamEntry` arrays live in static memory owned by their modules and are wired together into a global intrusive linked list with no dynamic allocation.

Parameter names are hashed at compile time using 32-bit FNV-1a (`ParamService::HashName`) for fast `FindByName()` lookups, while sequential `baseParamID` ranges are auto-assigned on registration for fast `FindByIndex()` telemetry iteration. All runtime writes through `Set()` enforce type matching (`ParamType::Float` vs `ParamType::Int32`), `[minVal, maxVal]` range bounds, and safety flag checks before modifying the bound parameter variable.

## Core Architecture
At system boot, `ParamService::Init()` registers the global `param_update` topic with the `Broker`. Each module then calls `ParamService::RegisterParams(&paramList, paramEntries)`, which initializes all bound variables to their factory defaults, assigns a contiguous block of sequential IDs, and links the `ParamList` into the registry.

When any parameter is modified, `Set()` sets its owning `ParamList::updated` atomic flag. Modules can then poll `ParamService::HasUpdate(&paramList)` (lock-free) to check if a parameter was updated and, if required, update derived variables such as filter coefficients. When `notify = true` (the default), `Set()` also publishes a `ParamUpdateMsg` on `TopicID::ParamUpdate` (`0x00C8`) via the [PubSub](./pubsub.md) bus, where it can be logged to SD and forwarded as a parameter changed ACK over [InstinctLink](./instinctlink.md).

Each parameter has a access flag, indicating what kind of changes (if any) are allowed to be done to it. This prevents changes to parameters that can lead to catastrophic failure of the associated Module. There are four levels of access:

| Flag | Mask | Behavior |
| :--- | :--- | :--- |
| `None` | `0x00` | Modifiable at any time |
| `ReadOnly` | `0x01` | Rejects all runtime `Set()` calls |
| `Protected` | `0x02` | Rejected while `ParamService::SetProtectedLock(true)` is active (e.g. armed or motors active) |
| `Volatile` | `0x04` | Marks parameters tuned dynamically at runtime (e.g. auto-tune) |

Calling `ParamService::Reset()` restores all registered parameters across every module to their `defaultVal`, marks every `ParamList` as updated, and publishes a bulk notification (`paramIndex = 0xFFFF`, `name = "RESET_ALL"`). `Reset()` is blocked while the `Protected` lock is active.

**Lifetime Note:** Because `ParamService` stores raw pointers in an intrusive linked list, both `ParamList` and the bound module variables must be `static` or live for the entire lifetime of the firmware. The `ParamEntry` array must end with a default-constructed sentinel entry (`ParamService::ParamEntry()`).

## Example Code

```cpp
#include "paramService.hpp"

class RateController {
	public:
		static void Init() {
			// Links descriptor array into registry and initializes variables to defaults
			ParamService::RegisterParams(&paramList, paramEntries);
		}

		static void Update() {
			// Lock-free check: only recompute derived coefficients when a parameter changed
			if(ParamService::HasUpdate(&paramList)) {
				RecomputeFilters();
			}
			// Use rollP and filterCutoffHz directly in the control loop
		}

	private:
		static inline float rollP = 0.0f;
		static inline int32_t filterCutoffHz = 0;

		static inline ParamService::ParamList paramList{};

		// Must be null-terminated with a default-constructed sentinel entry
		static constexpr ParamService::ParamEntry paramEntries[] = {
			ParamService::ParamEntry("CTRL_ROLL_P", &rollP, 0.15f, 0.0f, 2.0f),
			ParamService::ParamEntry("CTRL_GYRO_LPF", &filterCutoffHz, 80, 10, 250, ParamFlags::Protected),
			ParamService::ParamEntry() // Sentinel
		};
};
```