# R8 liquid U3 C1 GenCase bootstrap/runtime profile v2 -- STATIC DRAFT.
#
# NO_GO_PENDING_FD_TRANSPORT_AND_SIGNAL_PROBE.  This fresh v2 identity is not a
# replacement/loadable production profile yet.  In particular, bubblewrap
# transition/signal rules remain production-NO-GO until a harmless probe proves
# timeout TERM/KILL and lowercase rpx behavior on this host.  Never
# fill those rules by analogy and never load this draft for GenCase.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-u3-gencase-bootstrap-v2 flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /usr/bin/setpriv rix,
  /usr/bin/sha256sum rix,
  /work/runtime/GenCase_linux64 rpx -> r8-liquid-u3-gencase-runtime-v2,

  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /work/ r,
  /work/runtime/ rw,
  /work/runtime/** rw,
  /work/output/ rw,
  /work/output/** rw,

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
  ptrace (read, readby) peer=r8-liquid-u3-gencase-bootstrap-v2,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-gencase-bootstrap-v2,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  signal (send) set=(term,kill,exists) peer=r8-liquid-u3-gencase-runtime-v2,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent in this static-only v2 draft: production bwrap mount
  # grammar.  A successor may freeze only already-probed base bwrap rules.
}

profile r8-liquid-u3-gencase-runtime-v2 flags=(attach_disconnected,mediate_deleted) {
  /work/runtime/GenCase_linux64 rm,
  /work/runtime/DsphConfig.xml r,
  /work/runtime/C1_static_Def.xml r,
  / r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /work/output/ rw,
  /work/output/** rw,

  signal (receive) set=(term,kill,exists) peer=r8-liquid-u3-gencase-bootstrap-v2,
  signal (receive) set=(term,kill,exists) peer=unconfined,

  # No userns, mount, capability, network, /proc,
  # /dev, /home, /usr/bin exec, ptrace, change_profile or self-exec rule.
}
