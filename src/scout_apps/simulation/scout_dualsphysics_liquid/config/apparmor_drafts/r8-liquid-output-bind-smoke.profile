# R8 liquid target AppArmor review draft.
# DRAFT_ONLY: NOT_APPROVED_FOR_LOADING
# This named profile is a review artifact and has no attachment path.  It must
# not be copied to /etc/apparmor.d, parsed, loaded, enabled, or selected by a
# launcher without separate administrator approval.
#
# Its scope is only the future one-marker output-bind smoke.  It is deliberately
# non-operational: no executable file, mount, umount, remount, capability,
# signal, ptrace, dbus, unix, mqueue, or network permission is present.  It
# therefore cannot authorize bubblewrap and must fail closed if selected.
#
# No broad filesystem rule and no reusable unconfined template are included.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-output-bind-smoke flags=(attach_disconnected,mediate_deleted) {
  # This is the subject of a future independent administrator review, not an
  # approved activation grant.
  userns,

  # Default-deny applies to every other path.  These are the only future
  # marker-path declarations; neither rule grants an executable or a mount.
  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/build/ r,
  /home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/ r,
  /home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/ w,
  /home/zrj/scout_liquid_lab/build/u3_bwrap_output_bind_smoke_v1_*/output/.r8_output_bind_smoke_v1 rw,
}
