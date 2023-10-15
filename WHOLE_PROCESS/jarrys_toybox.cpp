

int main()
{
	char  mode[4] = { '8', 'N', '1', 0 };
	RS232 rs232(3, 115200, mode, false);
	
	rs232.run();
	
	clock_t start;
	
	start = clock();
	
	while (clock() - start < 3000000)
	{
//        rs232.set_velocity(5);
//        rs232.set_gear(GEAR_BACKWARD);
//        if(clock() - start > 15000) rs232.set_break(200);
		std::cout << rs232.get_velocity() << std::endl;
		
	}
}
