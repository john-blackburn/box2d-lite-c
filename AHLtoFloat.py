import sys

xs=[
#seeeeeeemmmmmmmmmmmmmmmm
'010000001001001000100000',  # 3.14156
'101111100000000000000000',  # -0.5
'000000010000000000000000',  # 2.16841e-19
'011111101111111111111111',  # 1.84466e19
'001111110000000000000000',  # 1.0
'010000000000000000000000']  # 2.0

xs0=[3.14156, -0.5, 2.16841e-19, 1.84466e19, 1.0, 2.0] # correct values

def convert(ss, exps, mans):
    man=1
    n=0
    for c in mans:
        n+=1
        if c=='1':
            man += 1/(2**n)

    exp = int(exps,2)-63

    value = man*2**exp
    sign='+'
    if (ss=='1'):
        value = -value
        sign = '-'

    print (sign, man, '* 2 ^', exp, '=', value)

unit_test=False

if unit_test:
    i=0
    for x in xs:
        ss=x[0]
        exps=x[1:8]
        mans=x[8:]
        
        convert(ss, exps, mans)
        print("Should be:",xs0[i])
        print()
        i+=1
        
if len(sys.argv)<2:
    print ("usage pyhton AHLtoFloat.py ABCDEF BAADF0 ...")
    sys.exit(1)

for arg in sys.argv[1:]:
    print()

    x=int(arg,16)

    b=bin(x)
    b=b[2:]

    while (len(b) < 24):
       b='0'+b

    print(hex(x), b[0], b[1:8], b[8:], "(", len(b), "bits )")

    ss=b[0]
    exps=b[1:8]
    mans=b[8:]

    convert(ss, exps, mans)
