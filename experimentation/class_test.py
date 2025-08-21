from dataclasses import dataclass, field

@dataclass
class A:
    prop1: int
    prop2: str
    prop3: list

@dataclass
class B:
    prop_a: A

def main():
    a = A(prop1=0, prop2="abc", prop3=[1,2,3])
    print(f"a={a}")

    b1 = B(prop_a=a)
    print(f"b1={b1}")

    b2 = B(prop_a=b1.prop_a)
    print(f"b2={b2}")

    b1.prop_a.prop3.append(4)
    a.prop3.append(5)

    a.prop1=1

    print("-----------")
    print(f"b1={b1}")
    print(f"b2={b2}")
    print(f"a={a}")

if __name__ == "__main__":
    main()