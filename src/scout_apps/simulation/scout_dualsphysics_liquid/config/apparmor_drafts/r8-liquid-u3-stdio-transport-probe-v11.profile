# R8 liquid U3 harmless stdio/AppArmor exact proc-mount success probe v11.
#
# FRESH_SINGLE_ATTEMPT_DRAFT: STATIC REVIEW DOES NOT AUTHORIZE EXECUTION.
# This named-only profile pair has no attachment path and must never be copied
# into /etc/apparmor.d.  A future explicitly admitted one-shot supervisor may
# load these exact root-owned snapshot bytes and must remove both labels in the
# same lifecycle.
#
# The root setup and read-only /usr rules below are byte-for-byte subsets of
# the locally successful u2 ro-/usr v8 profile.  The /work tmpfs rule and both
# synthetic lib links are byte-for-byte subsets of the locally successful U3
# output/CPU profiles.  Frozen v10 proved that the exact proc mount succeeds,
# then recorded a non-fatal dac_override check, avoidable Python site/timezone
# reads, and a fatal NNP rejection of the cross-profile sleep transition.  v11
# grants none of those denied accesses: it keeps dac_override explicitly
# denied, prevents the avoidable reads in fixed argv/env, and executes sleep by
# inheritance.  The child remains empty-group, zero-capability, NNP, inside the
# network-isolated namespace with nested userns disabled.
profile r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /usr/bin/sleep rix,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  /work/ rw,
  /work/input/ rw,
  /work/input/probe_alpha.bin rw,
  /work/input/probe_beta.bin rw,
  /work/input/probe_gamma.bin rw,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,

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
  ptrace (read, readby) peer=r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: /newroot/proc/**, every other proc mount rule, /dev, host
  # writable bind, --file/--bind-fd surface, workspace/source/seed paths,
  # GenCase/solver/ROS/Gazebo/GPU executables or devices, and generic mount.
}

profile r8-liquid-u3-stdio-transport-runtime-v11-20260808t052940z flags=(attach_disconnected,mediate_deleted) {
  /usr/bin/sleep rm,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,

  signal (receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v11-20260808t052940z,
  signal (receive) set=(term,kill,exists) peer=unconfined,

  # Intentionally unreachable in v11: fixed argv uses rix and the profile has
  # no attachment path.  Retained only so cleanup still proves both fresh
  # labels absent.  No write, userns, capability, network, proc, dev, home,
  # workspace, interpreter execution, profile-change or self-execution.
}
