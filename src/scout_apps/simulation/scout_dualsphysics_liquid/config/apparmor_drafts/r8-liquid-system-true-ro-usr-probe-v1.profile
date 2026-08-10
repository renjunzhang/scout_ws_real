# R8 liquid target transient AppArmor read-only /usr bind probe v1.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# Scope: reproduce the already-audited empty-root bwrap setup, then request one
# future read-only /usr bind for the fixed /usr/bin/true child. This v1 grants
# source-directory metadata read only; it intentionally contains NO /usr bind
# mount rule, so the first run must fail closed before any host tree enters the
# synthetic root. It has no host writable bind, source/workspace path, output,
# generic mount or umount, upstream executable, network stream, GPU, or
# persistent-install permission. It must never be copied to /etc/apparmor.d.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-system-true-ro-usr-probe-v1 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  # The only executable chain is aa-exec -> bwrap -> system true.
  /usr/bin/bwrap rix,
  /usr/bin/true rix,

  # Trusted loader paths for the two fixed system programs. `/usr/ r` permits
  # only the bind source directory lookup; no recursive /usr host data path is
  # readable merely because of this rule.
  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /dev/null rw,
  /dev/zero r,
  /dev/random r,
  /dev/urandom r,

  # Exact internal empty-root setup observed in v1-v11. All paths below are
  # created by bwrap inside its new mount namespace; they are not host binds.
  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,
  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,
  pivot_root oldroot=/tmp/oldroot/ /tmp/,
  mount options=(rw, silent, rprivate) -> /oldroot/,
  umount /oldroot/,
  pivot_root oldroot=/newroot/ /newroot/,
  umount /,

  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,

  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-system-true-ro-usr-probe-v1,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: any mount rule whose source is /usr, every bind or
  # rbind rule, remount, generic umount, source/workspace/output rule, broad
  # host read/write, /proc mount, /dev mount, stream socket, signal, dbus,
  # unconfined transition, or upstream executable permission.
}
