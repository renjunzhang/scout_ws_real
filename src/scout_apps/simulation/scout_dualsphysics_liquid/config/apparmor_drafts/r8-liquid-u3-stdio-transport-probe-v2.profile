# R8 liquid U3 stdio/AppArmor transport probe v2 -- STATIC NO-GO DRAFT.
#
# STATIC_REVIEW_ONLY: DO_NOT_PARSE_LOAD_SELECT_OR_EXECUTE.
# This wholly separate named profile pair has no attachment path.  It is a
# review artifact for a future harmless probe containing only fixed synthetic
# bytes and root-owned Ubuntu tools.  It grants no GenCase, solver, workspace,
# source, ROS, GPU, network-stream, host-output or host-writable path.
#
# Bubblewrap filesystem-mediation rules are intentionally absent.  They must
# be learned one denial at a time from a separately admitted harmless attempt,
# recorded with sanitized audit evidence, and frozen only in a fresh successor
# identity.  Nothing in this draft authorizes an attempt or a production run.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /usr/bin/sleep rpx -> r8-liquid-u3-stdio-transport-runtime-v2-20260807t141934z,

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

  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  signal (send) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-runtime-v2-20260807t141934z,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally no filesystem-mediation rule is present in this revision.
}

profile r8-liquid-u3-stdio-transport-runtime-v2-20260807t141934z flags=(attach_disconnected,mediate_deleted) {
  /usr/bin/sleep rm,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,

  signal (receive) set=(term,kill,exists) peer=r8-liquid-u3-stdio-transport-bootstrap-v2-20260807t141934z,
  signal (receive) set=(term,kill,exists) peer=unconfined,

  # No write, userns, capability, network, proc, dev, home, workspace,
  # interpreter execution, profile-change or self-execution authority.
}
