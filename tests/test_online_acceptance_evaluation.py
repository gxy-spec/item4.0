import pytest
from scripts.analyze_online_acceptance import classify_commit


@pytest.mark.parametrize('evidence,expected',[
    ({'matched_actor_id':None,'semantic_correct':False},'unverified'),
    ({'matched_actor_id':28,'semantic_correct':False},'wrong_target'),
    ({'matched_actor_id':26,'semantic_correct':True,'localization_error_m':6.},'position_rejected'),
    ({'matched_actor_id':26,'semantic_correct':True,'localization_error_m':2.},'accepted'),
])
def test_unresolved_association_is_not_a_proven_wrong_target(evidence,expected):
    assert classify_commit(evidence)==expected
