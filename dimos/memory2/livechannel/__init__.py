# Backwards-compat shim: livechannel/ was renamed to notifier/.
# Kept so that deserialize_component() can resolve old registry entries.
from dimos.memory2.notifier import Notifier, SubjectNotifier

__all__ = ["Notifier", "SubjectNotifier"]
