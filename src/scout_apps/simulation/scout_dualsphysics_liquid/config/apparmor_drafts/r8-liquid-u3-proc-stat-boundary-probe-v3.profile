# R8 liquid U3 harmless AppArmor numeric-proc-stat boundary probe v3.
# Identity: u3_proc_stat_apparmor_boundary_probe_v3_20260809T022402Z.
#
# FRESH_SINGLE_ATTEMPT_DRAFT: STATIC REVIEW DOES NOT AUTHORIZE EXECUTION.
# This named-only profile pair has no attachment path and must never be copied
# into /etc/apparmor.d.  A future explicitly admitted one-shot supervisor may
# load these exact root-owned snapshot bytes and must remove both labels in the
# same lifecycle.
#
# The complete effective-authority baseline is frozen cold-A v6 after exact
# profile-label normalization.  Relative to that full baseline, the sole new
# allow is the bootstrap-only numeric-PID stat read below.  Every other change
# is an authority removal, a fresh-label substitution, or a comment change.
# Status remains deliberately absent so one dumpable-zero child produces one
# bounded, expected DENIED record.  Frozen v6 already proved that the pinned
# system Python starts while the dynamic-linker cache remains quietly denied.
profile r8-liquid-u3-proc-stat-boundary-bootstrap-v3-20260809t022402z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  deny /etc/ld.so.cache r,

  /work/ rw,

  /proc/ r,
  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/[0-9]*/stat r,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,
  /proc/stat r,

  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,
  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,
  pivot_root oldroot=/tmp/oldroot/ /tmp/,
  mount options=(rw, silent, rprivate) -> /oldroot/,
  umount /oldroot/,
  pivot_root oldroot=/newroot/ /newroot/,
  umount /,
  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,
  mount fstype=proc options=(rw, nosuid, nodev, noexec) proc -> /newroot/proc/,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib wl,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/proc/ rw,

  deny capability dac_override,
  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-proc-stat-boundary-bootstrap-v3-20260809t022402z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-proc-stat-boundary-bootstrap-v3-20260809t022402z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: every other non-owner /proc child rule (especially
  # status), /newroot/proc/**, /dev, host writable bind, --file/--bind-fd,
  # workspace/source/seed paths, GenCase/solver/ROS/Gazebo/GPU executables or
  # devices, and generic mount.
}

profile r8-liquid-u3-proc-stat-boundary-runtime-v3-20260809t022402z flags=(attach_disconnected,mediate_deleted) {
  # Deliberately unreachable and empty.  It exists only so the lifecycle must
  # prove both fresh labels absent after cleanup.
}
