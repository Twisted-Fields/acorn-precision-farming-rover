import redis
import time
import pickle

r = redis.Redis(
    host='localhost',
    port=6379)

# r.set('foo', 'bar')


for key in r.scan_iter():
       print(key)
       # if 'twistedfields:acorn1' in str(key):
       #     print(key)
       # #     newkey = str(key).replace('-key\'',':key')
       # #     newkey = newkey.replace('b\'','')
       # #     print(newkey)
       # #     # #print(bytes(newkey, encoding='ascii'))
       # #     # # #newkey = "twistedfields:gpspath:{}-key".format(str(key))
       #     r.delete(key)
       # try:
       #     value = pickle.loads(r.get(key))
       #     print(value)
       # except:
       #     print('exception unpickling key {}'.format(key))
       #     r.delete(key)



# while True:
#     value = r.get('foo')
#     print(value)
#     time.sleep(0.1)
