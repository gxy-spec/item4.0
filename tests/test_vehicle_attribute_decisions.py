from pathlib import Path
from types import SimpleNamespace
import numpy as np
from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig


class Attributes:
    def __init__(self, category='van', score=.98, color='blue'):
        self.category=category;self.score=score;self.color=color

    def predict(self, image, boxes):
        # The interface contains neither a task nor simulator actor metadata.
        return [dict(predicted_type=self.category,type_probability=self.score,
            type_probabilities={self.category:self.score},predicted_color=self.color,
            color_probability=.99,color_probabilities={k:(.99 if k==self.color else .01) for k in ['red','blue','white','black','gray','yellow']},
            attribute_source='test_rgb_classifier') for _ in boxes]


def pipeline():
    return CandidatePipeline(PipelineConfig(model_path=Path('best.pt'),detector_backend='custom',
        enable_attribute_proposals=False),model=SimpleNamespace(names={0:'van',1:'car',2:'truck',3:'bus'}))


def run(p):
    image=np.zeros((600,800,3),np.uint8);image[:,:,2]=255
    result=SimpleNamespace(names={0:'van',1:'car'},boxes=SimpleNamespace(
        xyxy=np.array([[200,200,350,350]],float),conf=np.array([.99]),cls=np.array([1])))
    return p.process(1,.1,image,np.full((600,800),20.),
        {'location':[0.,0.,30.],'rotation':[-90.,0.,0.]},result)[0]


def test_learned_vehicle_type_overrides_detector_but_retains_provenance():
    p=pipeline();p.attribute_recognizer=Attributes(color='red');c=run(p)
    assert c['category']=='van' and c['detector_category']=='car'
    assert c['category_match_score']==1 and c['target_color_match']


def test_learned_color_decides_even_when_hsv_disagrees():
    p=pipeline();p.attribute_recognizer=Attributes();c=run(p)
    assert c['red_pixel_ratio']>.9 and c['color']=='blue'
    assert not c['target_color_match'] and c['target_color_probability']==.01


def test_uncertain_type_is_unknown_and_cannot_confirm_target():
    p=pipeline();p.attribute_recognizer=Attributes(score=.4,color='red');c=run(p)
    assert c['category']=='unknown' and c['category_match_score']==0
    assert not c['temporal_confirmed']
