# R8 liquid target transient AppArmor mount-confinement probe v2.
# EPHEMERAL_PROBE_ONLY: LOAD_FROM_WORKSPACE_AND_REMOVE_IN_THE_SAME_SESSION.
# This named profile has no attachment path, so it cannot affect an existing
# process unless that process explicitly selects this exact profile.
#
# Scope: allow only the two observed bwrap setup mounts after it creates its
# own user and mount namespaces: root rslave propagation and a nodev/nosuid
# tmpfs at /tmp/.  Bubblewrap 0.9.0 documents that it always creates a new
# mount namespace.  This profile never permits a bind, remount, umount, procfs,
# devtmpfs, output bind, source bind, or host-writable mount.  It must be
# removed immediately after the bounded probe and must never be copied to
# /etc/apparmor.d or enabled for boot-time loading.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-system-true-mount-probe-v2 flags=(attach_disconnected,mediate_deleted) {
  # Required by kernel.apparmor_restrict_unprivileged_userns=1.  This is an
  # explicit create permission, not an unconfined escape.
  userns create,

  # The only executable chain is aa-exec -> bwrap -> system true.  The
  # timeout and resource limits remain outside this named profile.
  /usr/bin/bwrap rix,
  /usr/bin/true rix,

  # Read/map only the dynamic loader and libraries needed by those fixed,
  # root-owned system binaries.  No executable wildcard is granted.
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /etc/ld.so.conf r,
  /etc/ld.so.conf.d/** r,

  # bwrap may inspect its own proc records, read the filesystem list and two
  # kernel overflow IDs, write the UID/GID maps and the required "deny" value
  # to setgroups for its own just-created child user namespace, and construct
  # a synthetic /dev.  The kernel still validates those mappings; this does
  # not grant a writable host filesystem path.
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

  # These two rules exactly match the observed bwrap setup sequence.  The
  # first makes the root propagation private to bwrap's mount namespace; the
  # second creates only an ephemeral, nodev/nosuid tmpfs at /tmp/ there.
  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,

  # The kernel grants these capabilities only after bwrap has entered its new
  # user namespace; they do not create host-namespace capabilities.  bwrap
  # also uses only datagram sockets while bringing up loopback in the new,
  # explicitly unshared network namespace.  There is no stream socket rule.
  capability sys_admin,
  capability sys_ptrace,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-system-true-mount-probe-v2,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # No generic mount, bind/rbind/move/remount/umount, procfs, devtmpfs,
  # source, workspace, output-bind, ROS, GPU, broad host write,
  # change_profile, signal, dbus, unix IPC, mqueue, or upstream executable
  # permission is present.  Any operation after the two exact setup mounts
  # must fail closed and be independently reviewed.
}
