from HADA import RS232_py

if __name__ == "__main__":
    rs232 = RS232_py.RS232(4)
    rs232.run()
    i = 0

    while(True):
        vel = rs232.get_velocity()
        print(f"{i}: {vel}")
        i += 1
    