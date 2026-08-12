#!/usr/bin/env python3
"""Static/synthetic tests for single-bag S6 analysis and delivery v7."""

from __future__ import annotations
import ast,copy,math,sys,tempfile,unittest
from pathlib import Path
from PIL import Image
from unittest import mock
ROOT=Path(__file__).resolve().parent.parent;SCRIPT=ROOT/'scripts/r8_liquid_s6_primary_analysis_delivery_v7.py';sys.path.insert(0,str(ROOT/'scripts'))
import r8_liquid_s6_primary_analysis_delivery_v7 as runtime  # noqa:E402
import r8_liquid_s6_real_runtime_transaction_v7 as transaction  # noqa:E402

class S6PrimaryAnalysisV7Tests(unittest.TestCase):
 def surface(self):
  times=[i*.5 for i in range(80)];rows=[]
  for t in times:
   heights=[.058+.003*math.exp(-.02*t)*math.sin(2*math.pi*.1*t+p*math.pi/8) for p in range(16)];eta=[v-.058 for v in heights]
   rows.append({'time_s':t,'H_crest_m':max(eta),'H_abs_m':max(abs(v) for v in eta),'H_peak_to_peak_m':max(heights)-min(heights),'valid_probe_count':16,'probe_heights_m':heights})
  return {'time_grid_sha256':runtime.sha256_json(times),'rows':rows,'probe_names':[f's5b0_p{i:02d}' for i in range(16)],'h0_m':.058,'fact_source':'SIXTEEN_RAW_NATIVE_JGAUGESWL_CSV'}
 def selected(self):
  def series(name):
   samples=[]
   for i in range(4,72):
    t=i*.5;value=.0027*math.exp(-.019*t)*math.sin(2*math.pi*.1*t+.05)
    native=value if name=='H_proxy' else value*1000
    stamp=int(t*1e9)+1
    samples.append({'bag_record_t_ns':stamp,'mapped_odom_header_t_ns':stamp,
     'time_since_odom_origin_s':t,'value_native':native,'value_comparison_mm':value*1000})
   return {'topic':'/slosh/height' if name=='H_proxy' else '/spmpc/slosh_height',
    'message_type':'std_msgs/Float32','native_unit':'m' if name=='H_proxy' else 'mm',
    'comparison_unit':'mm','scale_to_comparison':1000.0 if name=='H_proxy' else 1.0,
    'offset_to_comparison':0.0,'sample_count':len(samples),'samples':samples}
  proxy=series('H_proxy');modal=series('H_modal')
  identity=lambda name,index:{'path':f'/fixture/{name}','sha256':'a'*64,'size_bytes':1,
   'mode':'0400','device':1,'inode':index,'nlink':1,'mtime_ns':1,'ctime_ns':1}
  return {'schema_version':'smpcc-r8-liquid-s6-primary-selected-signals-v7',
   'document_type':'SMPCC_R8_LIQUID_S6_PRIMARY_SELECTED_SIGNALS_V7',
   'status':'PASS_S6_PRIMARY_SELECTED_SIGNALS_V7_READ_ONLY','attempt_id':runtime.ATTEMPT_ID,
   'planned_denominator':1,'source_outcome':'UNKNOWN',
   'parents':{'s5a0_selected_bag_receipt':identity('s5a0',1),'s5a1_transfer_manifest':identity('s5a1',2),'source_bag':identity('bag',3)},
   'reader_contract':{'extractor':{'relative_path':'scripts/r8_liquid_s6_real_selected_signal_extractor_v5.py','sha256':'a'*64},
    'reader_core':{'relative_path':'scripts/r8_liquid_ros1_bag_v2_reader_v1.py','sha256':'b'*64},
    'reader_v4':{'relative_path':'scripts/r8_liquid_ros1_bag_v2_reader_v4.py','sha256':'c'*64},
    'extractor_v3':{'relative_path':'scripts/r8_liquid_s5a1_ros1_signal_extractor_v3.py','sha256':'d'*64},
    'input_surface':'IMMUTABLE_BOUNDED_EXACT_PRIMARY_BAG_BYTES_ONLY'},
   'time_alignment':{'x_axis':'time_since_odom_header_origin_s','motion_time_source':'/odom.header.stamp',
    'signal_native_time_source':'ROS1_BAG_RECORD_TIME_NS','mapping_method':'LOWER_MEDIAN_ODOM_HEADER_MINUS_RECORD_OFFSET_V1',
    'odom_header_origin_ns':1,'odom_header_end_ns':40000000000,'offset_sample_count':68,
    'record_to_odom_header_offset_ns':0,'residual_mean_ns':0.0,'residual_rms_ns':0.0,
    'residual_max_abs_ns':0,'maximum_allowed_residual_ns':5000000,'residuals_sha256':'e'*64,
    'mapping_extrapolation':False,'overlap_start_s':2.0,'overlap_end_s':35.5},
   'series':{'H_proxy':proxy,'H_modal':modal},
   'integrity':{'source_bag_sha256':'a'*64,
    'H_proxy_samples_sha256':runtime.sha256_bytes(runtime.canonical_json(proxy['samples'])),
    'H_modal_samples_sha256':runtime.sha256_bytes(runtime.canonical_json(modal['samples'])),
    'reader_anomalies_absent':True,'parents_unchanged':True},
   'claims':{'read_only':True,'comparison_only':True,'optional_bag_read':False,
    'source_bag_executed':False,'ros_started':False,'motion_exporter_consumed_selected_signals':False,
    'solver_forcing_consumed_selected_signals':False,'stage6_pass':False,'development_only':True,
    'paired_ranking':False,'cross_method_ranking':False,'selected_trajectory_cpu_comparison':False,
    'physical_reference_pending':True,'physical_fidelity_validated':False,'formal':False,'production':False}}
 def windows(self):return {'first15':{'start_index':0,'end_index':30,'start_s':0.0,'end_s':15.0},'full_motion':{'start_index':0,'end_index':40,'start_s':0.0,'end_s':20.0},'recorded_tail':{'start_index':40,'end_index':60,'start_s':20.0,'end_s':30.0},'solver_tail':{'start_index':60,'end_index':79,'start_s':30.0,'end_s':39.5}}
 def test_analysis_four_windows_metrics_na_and_no_ranking(self):
  result=runtime.analyze(self.surface(),self.selected(),self.windows());self.assertEqual(set(runtime.WINDOWS),set(result['window_metrics']));self.assertEqual(6,len(result['comparisons']));self.assertTrue(all(not row['ranking_claimed'] for row in result['comparisons']));self.assertTrue(any(row['H_proxy_m'] is None for row in result['solver_rows']));self.assertFalse(result['claims']['cross_method_ranking']);self.assertEqual(.058,result['h0_m'])
 def test_windows_claim_and_source_outcome_fail_closed(self):
  changed=self.windows();changed['first15']['start_index']=1
  with self.assertRaises(runtime.S6PrimaryV7Error):runtime.analyze(self.surface(),self.selected(),changed)
  selected=self.selected();selected['source_outcome']='SPMPC_NON_FIXED'
  with self.assertRaises(runtime.S6PrimaryV7Error):runtime.analyze(self.surface(),selected,self.windows())
  for path in (('planned_denominator',),('claims','optional_bag_read'),('claims','cross_method_ranking')):
   selected=copy.deepcopy(self.selected());cursor=selected
   for key in path[:-1]:cursor=cursor[key]
   cursor[path[-1]]=2 if path==('planned_denominator',) else True
   with self.assertRaises(runtime.S6PrimaryV7Error):runtime.analyze(self.surface(),selected,self.windows())
 def test_transaction_rejects_unbound_fixture_before_any_staging(self):
  with tempfile.TemporaryDirectory() as temporary:
   base=Path(temporary);root=base/'delivery';spec=transaction.TransactionSpec(transaction_id='s6-v7-one-bag-fixture',runtime_contract_sha256='a'*64,expected_previous_ledger_sha256='0'*64,partial_root=root.with_name(root.name+'.partial'),final_root=root,ledger_path=base/'ledger.jsonl',final_receipt_path=base/'receipt.json')
   with self.assertRaises(Exception):runtime.publish(spec,{'comparison_manifest.json':b'{}\n'},{},{})
   self.assertFalse(spec.partial_root.exists());self.assertFalse(spec.final_root.exists())

 def test_renderer_exports_svg_and_programmatic_qa_stays_separate_from_visual_review(self):
  analysis=runtime.analyze(self.surface(),self.selected(),self.windows())
  figures=runtime.render_three_panel(analysis)
  self.assertEqual({'figures/primary_shared_x_timeseries.png','figures/primary_shared_x_timeseries.pdf','figures/primary_shared_x_timeseries.svg','figures/primary_shared_x_timeseries_grayscale.png'},set(figures['artifacts']))
  self.assertTrue(figures['qa']['svg_render_pass']);self.assertTrue(figures['qa']['no_clipping']);self.assertFalse(figures['qa']['multimodal_visual_review'])
  self.assertIn(b'<svg',figures['artifacts']['figures/primary_shared_x_timeseries.svg'][:1024])

 def test_media_complete_decode_dimensions_fps_duration_and_keyframes(self):
  import io
  frames=[];manifest=[]
  for index in range(5):
   image=Image.new('RGB',(160,96),(20+index*20,80,140));stream=io.BytesIO();image.save(stream,format='PNG');raw=stream.getvalue();frames.append(raw);manifest.append({'index':index,'time_s':index*.1,'source_bi4_sha256':f'{index+1:064x}','rendered_png_sha256':runtime.sha256_bytes(raw),'probe_grid_sha256':'a'*64,'attachment_frame':'MOVING_CONTAINER_REFERENCE_REF_0'})
  rendered={'frames':frames,'manifest':manifest,'frame_manifest_sha256':'b'*64,'numeric_fact_source':False}
  media=runtime.encode_media(rendered,fps=10)
  self.assertEqual(5,media['manifest']['frame_count']);self.assertAlmostEqual(.5,media['manifest']['duration_s']);self.assertAlmostEqual(10,media['manifest']['decoded_mp4_fps']);self.assertAlmostEqual(.5,media['manifest']['decoded_gif_duration_s'])
  self.assertTrue(media['qa']['keyframes_complete_decode']);self.assertEqual(3,len(media['manifest']['keyframes']))
 def test_real_composer_to_precommit_transaction_and_result_end_to_end(self):
  analysis=runtime.analyze(self.surface(),self.selected(),self.windows())
  figures=runtime.render_three_panel(analysis)
  import io
  frame_raw=[];frame_manifest=[]
  for index in range(5):
   image=Image.new('RGB',(160,96),(30+index*30,90,150));stream=io.BytesIO();image.save(stream,format='PNG');raw=stream.getvalue();frame_raw.append(raw);frame_manifest.append({'index':index,'time_s':index*.1,'source_bi4_sha256':f'{index+1:064x}','rendered_png_sha256':runtime.sha256_bytes(raw),'probe_grid_sha256':'a'*64,'attachment_frame':'MOVING_CONTAINER_REFERENCE_REF_0'})
  media=runtime.encode_media({'frames':frame_raw,'manifest':frame_manifest,'frame_manifest_sha256':'b'*64,'numeric_fact_source':False},fps=10)
  visual={'schema_version':'smpcc-r8-liquid-s6-multimodal-visual-qa-v7','status':'PASS_S6_MULTIMODAL_VISUAL_QA_V7','reviewed_preview_sha256':runtime.sha256_bytes(figures['artifacts']['figures/primary_shared_x_timeseries.png']),'reviewed_grayscale_sha256':runtime.sha256_bytes(figures['artifacts']['figures/primary_shared_x_timeseries_grayscale.png']),'no_clipping':True,'no_missing_glyphs':True,'no_legend_occlusion':True,'panel_alignment':True,'grayscale_distinguishable':True,'data_not_visually_clipped':True,'cross_panel_units_consistent':True}
  artifacts=runtime.build_artifacts(analysis,self.selected(),self.surface(),figures,media,visual)
  required=transaction.json.loads(transaction.ARTIFACT_BUNDLE_SCHEMA_PATH.read_bytes())['$defs']['requiredArtifacts']['const']
  self.assertEqual(required,sorted(artifacts))
  evidence=transaction.json.loads(artifacts['evidence_index.json'])
  self.assertEqual(sorted(set(artifacts)-{'evidence_index.json','checksums.sha256'}),[row['relative_path'] for row in evidence['entries']])
  from test_s6_real_runtime_transaction_v7 import S6RealRuntimeTransactionV7Tests
  helper=S6RealRuntimeTransactionV7Tests()
  with tempfile.TemporaryDirectory() as temporary:
   base=Path(temporary);root=base/'case';root.mkdir();final=root/'delivery'
   provisional=transaction.TransactionSpec(transaction_id='s6-v7-composer-e2e',runtime_contract_sha256='0'*64,expected_previous_ledger_sha256=transaction.ZERO_SHA256,partial_root=final.with_name(final.name+'.partial'),final_root=final,ledger_path=root/'ledger.jsonl',final_receipt_path=root/'receipt.json')
   contract=helper.runtime_contract(provisional);contract['analysis_windows']['time_grid_sha256']=self.surface()['time_grid_sha256'];contract['canonical_s5b0']['canonical_time_grid']['sha256']=self.surface()['time_grid_sha256'];contract['canonical_s5b0']['canonical_time_grid']['slot_count']=len(self.surface()['rows']);contract['canonical_s5b0']['frame_manifest']['frame_count']=len(self.surface()['rows'])
   for name,row in self.windows().items():contract['analysis_windows'][name]=row
   contract_sha=transaction.sha256_bytes(transaction.canonical_json(contract));spec=transaction.TransactionSpec(transaction_id=provisional.transaction_id,runtime_contract_sha256=contract_sha,expected_previous_ledger_sha256=provisional.expected_previous_ledger_sha256,partial_root=provisional.partial_root,final_root=provisional.final_root,ledger_path=provisional.ledger_path,final_receipt_path=provisional.final_receipt_path)
   canonical=contract['canonical_s5b0'];canonical_inputs={'finalized_frame_manifest':{'schema_version':canonical['frame_manifest']['schema_version'],'status':canonical['frame_manifest']['status'],'content_sha256':canonical['frame_manifest']['frame_manifest_sha256']},'external_inventory_receipt_sha256':canonical['frame_manifest']['external_inventory_receipt_sha256'],'expected_inventory_sha256':canonical['frame_manifest']['expected_inventory_sha256'],'canonical_ids_sha256':transaction.CANONICAL_IDS_SHA256,'particle_count':9078,'class_counts':canonical['frame_manifest']['class_counts'],'native_gauge_manifest':{'schema_version':canonical['gauge_manifest']['schema_version'],'status':canonical['gauge_manifest']['status'],'content_sha256':canonical['gauge_manifest']['native_gauge_manifest_sha256'],'probe_count':16,'attachment_frame':'MOVING_CONTAINER_REFERENCE_REF_0'},'time_grid_sha256':canonical['canonical_time_grid']['sha256'],'probe_grid_sha256':canonical['probe_grid_sha256'],'frame_reader_sha256':transaction.FRAME_READER_SHA256,'frame_schema_sha256':transaction.FRAME_SCHEMA_SHA256,'bi4_reader_sha256':canonical['bi4_reader']['sha256']}
   bundle=runtime.precommit_bundle(contract,artifacts,canonical_inputs)
   report=runtime.publish(spec,artifacts,contract,bundle)
   result=runtime.result_from_transaction(spec,report,contract_sha,bundle)
   self.assertEqual(runtime.FINAL_STATUS,result['status']);self.assertTrue(result['claims']['stage6_pass']);self.assertFalse(result['claims']['cross_method_ranking']);self.assertTrue(result['claims']['physical_reference_pending'])
   changed=copy.deepcopy(bundle);changed['quality']['evidence_index_sha256']='0'*64
   with self.assertRaisesRegex(runtime.S6PrimaryV7Error,'identity'):
    runtime.result_from_transaction(spec,report,contract_sha,changed)
   with self.assertRaisesRegex(runtime.S6PrimaryV7Error,'identity'):
    runtime.result_from_transaction(spec,report,'f'*64,bundle)
 def test_precommit_rejects_manifest_semantic_drift_before_staging(self):
  from test_s6_real_runtime_transaction_v7 import S6RealRuntimeTransactionV7Tests
  helper=S6RealRuntimeTransactionV7Tests()
  with tempfile.TemporaryDirectory() as temporary:
   case=helper.make_case(Path(temporary),'semantic-drift');spec,artifacts,contract,bundle=case
   changed=dict(artifacts);figure=transaction.json.loads(changed['reports/figure_manifest.json']);figure['dual_y_axes']=True;changed['reports/figure_manifest.json']=transaction.canonical_json(figure);changed['checksums.sha256']=''.join(f"{transaction.sha256_bytes(changed[name])}  {name}\n" for name in sorted(set(changed)-{'checksums.sha256'})).encode();bundle=helper.bundle(contract,changed)
   with self.assertRaisesRegex(transaction.S6TransactionV7Error,'figure manifest'):
    transaction.execute_transaction(spec,changed,contract,bundle)
   self.assertFalse(spec.partial_root.exists())
 def test_public_self_check_has_no_exec_or_optional_surface(self):
  report=runtime.self_check();self.assertEqual(1,report['planned_denominator'])
  for key in ('real_input_read','optional_bag_read','external_write','media_executed','candidate_executed','solver_or_gpu_executed','stage6_pass'):self.assertFalse(report[key])
  tree=ast.parse(SCRIPT.read_text());imports={a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names};imports.update(n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n,ast.ImportFrom) and n.module);self.assertFalse(imports&{'socket','requests','subprocess','rosbag','rospy'})

if __name__=='__main__':unittest.main()
