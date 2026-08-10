# R8 liquid U3 C1 GenCase v3, fresh single-attempt transient profile.
#
# Never install this file under /etc/apparmor.d.  The root-owned one-shot
# snapshot supervisor may load these exact bytes only for the frozen case ID,
# and must remove both exact labels in the same lifecycle.  GenCase executes
# with `rix`: under NoNewPrivs it inherits the bootstrap label already proven
# by the frozen v11 harmless stdio/AppArmor transport probe.  The second label
# is deliberately unreachable and exists only to keep cleanup evidence closed.
profile r8-liquid-u3-gencase-bootstrap-v3-20260808t062327z flags=(attach_disconnected,mediate_deleted) {
  userns create,

  /usr/bin/bwrap rix,
  /usr/bin/python3.12 rix,
  /usr/bin/setpriv rix,
  /usr/bin/sha256sum rix,
  /work/runtime/GenCase_linux64 rix,

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
  /work/** rw,

  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,

  # Exact bwrap mount grammar already demonstrated by frozen v11 with zero
  # logged denials.  No host writable bind, --file, --bind-fd or device mount.
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

  # v10 showed bwrap can harmlessly request this capability while still
  # succeeding.  Keep the exact quiet deny rather than broadening authority.
  deny capability dac_override,
  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer=r8-liquid-u3-gencase-bootstrap-v3-20260808t062327z,
  signal (send,receive) set=(term,kill,exists) peer=r8-liquid-u3-gencase-bootstrap-v3-20260808t062327z,
  signal (receive) set=(term,kill,exists) peer=unconfined,
  network unix dgram,
  network inet dgram,
  network inet6 dgram,
  network netlink raw,

  # Intentionally absent: host home/workspace/seed paths, writable host binds,
  # /dev, GPU, ROS/Gazebo, external network, solver execution, profile change,
  # persistent attachment, and generic mount authority.
}

profile r8-liquid-u3-gencase-runtime-v3-20260808t062327z flags=(attach_disconnected,mediate_deleted) {
  # Deliberately unreachable: GenCase uses rix and inherits the bootstrap
  # profile.  This label has no attachment, execution, read, write, mount,
  # userns, capability, network, proc, device, signal-send or profile-change
  # authority.  It is loaded only so lifecycle cleanup proves both fresh labels
  # absent before host export.
}
