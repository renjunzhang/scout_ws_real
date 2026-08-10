# R8 liquid U3 harmless stdio/AppArmor denial-discovery probe v7 successor.
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
# output/CPU profiles.  No proc mount or /newroot/proc permission is guessed:
# fixed argv deliberately reaches that next fail-closed discovery boundary.
profile r8-liquid-u3-stdio-transport-bootstrap-v7-20260807t175042z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /usr/bin/sleep rpx -> r8-liquid-u3-stdio-transport-runtime-v7-20260807t175042z,

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

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib wl,
  /newroot/lib64 wl,
  /newroot/work/ rw,

  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-stdio-transport-bootstrap-v7-20260807t175042z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v7-20260807t175042z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  signal (send) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-runtime-v7-20260807t175042z,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: /newroot/proc permission, proc mount rule, /dev,
  # host writable bind, --file/--bind-fd surface, workspace/source/seed paths,
  # GenCase/solver/ROS/Gazebo/GPU executables or devices, and generic mount.
}

profile r8-liquid-u3-stdio-transport-runtime-v7-20260807t175042z flags=(attach_disconnected,mediate_deleted) {
  /usr/bin/sleep rm,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,

  signal (receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v7-20260807t175042z,
  signal (receive) set=(term,kill,exists) peer=unconfined,

  # No write, userns, capability, network, proc, dev, home, workspace,
  # interpreter execution, profile-change or self-execution authority.
}
