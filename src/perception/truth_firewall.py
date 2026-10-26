"""Fail-closed actor facade for sensor-only online decision calls."""
class TruthAccessDenied(RuntimeError):pass


class DeniedActor:
    def __init__(self,actor,role,events):
        object.__setattr__(self,'_actor',actor);object.__setattr__(self,'_role',role);object.__setattr__(self,'_events',events)
    def __getattribute__(self,name):
        if name.startswith('_'):return object.__getattribute__(self,name)
        self._events.append(dict(role=self._role,attribute=name,denied=True))
        raise TruthAccessDenied('Online target/obstacle actor access denied: '+self._role+'.'+name)
