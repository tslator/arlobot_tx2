from halnode import HALNode, HALNodeError

def test_create():
    hn = HALNode()

    assert hn is not None
    
