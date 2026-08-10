# R8 liquid target source-only CPU-build review draft v1.
# DRAFT_ONLY: NOT_APPROVED_FOR_LOADING
# This named profile has no attachment path. It must not be copied to
# /etc/apparmor.d, parsed, loaded, enabled, or selected by a launcher without
# a separate administrator review and a separate execution admission.
#
# The body is deliberately non-operational: it has no executable, mount,
# umount, remount, capability, signal, ptrace, unix, dbus, mqueue, network, or
# unconfined permission. It therefore cannot authorize bubblewrap, a source
# copy, Make, G++, a linker, GenCase, a solver, or an output binary.
#
# These narrow host paths only freeze the future review surface: one already
# sealed source tree may be read, and one future build attempt subtree is the
# only prospective output location. Default deny applies everywhere else.
abi <abi/4.0>,

include <tunables/global>

profile r8-liquid-source-cpu-build-draft-v1 flags=(attach_disconnected,mediate_deleted) {
  userns,

  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/dependency/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/ r,
  /home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source/** r,
  /home/zrj/scout_liquid_lab/build/ r,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/output/ rw,
  /home/zrj/scout_liquid_lab/build/u3_source_cpu_build_*.partial/output/** rw,
}
