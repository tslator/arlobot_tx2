

'''
def rep_gen(iterable):
    def reset(gen):
        pass

    def _next(iterable):
        for i in iterable:
            print("before yield")
            yield i
            print("after yield")

    print("before value")
    value = _next(iterable)
    print("after value")

    return value
'''
#for i in rep_gen([1, 2, 3, 4]):
#    print(i)

'''
rg = rep_gen([1, 2, 3, 4])
print(rg)
'''

def myxrange(n):
    class Iterable(object):
        def __iter__(self):
            i = 0
            while i < n:
                yield i
                i += 1
    return Iterable()

myxrange(5)