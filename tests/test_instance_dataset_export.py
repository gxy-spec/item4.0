from __future__ import annotations

import numpy as np
import yaml
import cv2
import json

from scripts.export_instance_segmentation_dataset import (
    canonical_actor_color,
    category_from_actor,
    decode_actor_ids,
    largest_polygon,
    scene_identity,
    scene_split,
    vehicle_actor_mask,
)


def test_fixed_bus_material_uses_rendered_primary_body_color():
    assert canonical_actor_color({'type_id':'vehicle.mitsubishi.fusorosa','attributes':{'color':'0,0,255'}})=='white'
    assert canonical_actor_color({'type_id':'vehicle.tesla.model3','attributes':{'color':'0,0,255'}})=='blue'


def test_instance_id_decoder_selects_byte_order_using_actor_ids() -> None:
    encoded = np.zeros((4, 4, 3), dtype=np.uint8)
    actor_id = 513
    encoded[1:3, 1:3, 0] = actor_id % 256
    encoded[1:3, 1:3, 1] = actor_id // 256
    decoded, order = decode_actor_ids(encoded, [actor_id])
    assert order == "B_plus_256G"
    assert np.all(decoded[1:3, 1:3] == actor_id)


def test_instance_mask_converts_to_yolo_polygon() -> None:
    mask = np.zeros((20, 30), dtype=np.uint8)
    mask[5:15, 8:22] = 1
    polygon = largest_polygon(mask, 30, 20)
    assert polygon is not None
    assert len(polygon) >= 8
    assert all(0.0 <= value <= 1.0 for value in polygon)


def test_vehicle_semantics_prevents_nonvehicle_pixels_selecting_wrong_byte_order():
    encoded=np.zeros((10,10,3),dtype=np.uint8)
    encoded[:8,:,0]=2;encoded[:8,:,1]=1;encoded[:8,:,2]=3
    encoded[8:,:,0]=1;encoded[8:,:,1]=2;encoded[8:,:,2]=14
    decoded,order=decode_actor_ids(encoded,[513],semantic_tags=[14,15,16])
    assert order=='B_plus_256G'
    assert int(vehicle_actor_mask(decoded,encoded,513).sum())==20


def test_vehicle_mask_rejects_id_collision_on_nonvehicle_surface():
    encoded=np.zeros((4,4,3),dtype=np.uint8)
    encoded[:,:,2]=3;encoded[1,1,2]=15
    ids=np.full((4,4),26,dtype=np.uint16)
    mask=vehicle_actor_mask(ids,encoded,26)
    assert mask.sum()==1 and mask[1,1]


def test_blueprint_category_mapping() -> None:
    assert category_from_actor({"type_id": "vehicle.mercedes.sprinter"}) == "van"
    assert category_from_actor({"type_id": "vehicle.carlamotors.carlacola"}) == "truck"
    assert category_from_actor({"type_id": "vehicle.tesla.model3"}) == "car"
    assert category_from_actor({
        "type_id": "vehicle.mitsubishi.fusorosa",
        "attributes": {"base_type": "Bus"},
    }) == "bus"


def test_variants_of_same_physical_scene_share_split(tmp_path) -> None:
    keys = []
    for variant in ("V1", "V2"):
        run = tmp_path / variant / "run"
        run.mkdir(parents=True)
        config = {"simulation": {"map": "Town10HD"}, "region": {"region_id": "town10hd_east_1"}}
        (run / "resolved_config.yaml").write_text(yaml.safe_dump(config), encoding="utf-8")
        keys.append(scene_identity(run))
    assert keys[0] == keys[1]
    assert scene_split(keys[0]) == scene_split(keys[1])


def test_default_export_does_not_train_vehicle_on_a_building_id_collision(tmp_path,monkeypatch):
    from scripts.export_instance_segmentation_dataset import main
    run=tmp_path/'source'/'run';metadata=run/'sensors/ugv/metadata';metadata.mkdir(parents=True)
    (run/'actors').mkdir()
    (run/'actors/actor_states.jsonl').write_text(json.dumps({'frame':1,'actor_id':513,'type_id':'vehicle.tesla.model3'})+'\n',encoding='utf-8')
    rgb=run/'rgb.png';instance=run/'instance.png'
    cv2.imwrite(str(rgb),np.zeros((20,20,3),np.uint8))
    encoded=np.zeros((20,20,3),np.uint8);encoded[:,:,0]=1;encoded[:,:,1]=2;encoded[:,:,2]=3
    cv2.imwrite(str(instance),encoded)
    (metadata/'frames.jsonl').write_text(json.dumps({'frame':1,'rgb_path':str(rgb),'instance_segmentation_path':str(instance)})+'\n',encoding='utf-8')
    strict=tmp_path/'strict';legacy=tmp_path/'legacy'
    monkeypatch.setattr('sys.argv',['export',str(run),'--output',str(strict)])
    assert main()==0
    assert (strict/'manifest.jsonl').read_text(encoding='utf-8')==''
    monkeypatch.setattr('sys.argv',['export',str(run),'--output',str(legacy),'--legacy-unfiltered-actor-ids'])
    assert main()==0
    assert len((legacy/'manifest.jsonl').read_text(encoding='utf-8').splitlines())==1
