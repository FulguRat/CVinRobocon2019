#include "getStatus.h"

#ifdef __linux__
void GetStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround)
{
	uint8_t serialData[8];
	int step = 0;
	int wileFlag = 1;

	while(wileFlag)
	{
          std::cout <<"step:" << step << std::endl;
	  switch(step)
	  {
	    case 0:
	    my_serial->read(serialData,8);
	    step = 1;
	    break;
	    case 1:
	    if(serialData[0] == 'a' && serialData[1] == 't' && serialData[2] == '+')
		{
			step = 2;
		}
	    else{
		        step = 0;
		        std::cout << "wait...\n";
                        my_serial->write("wait..\r\n");
		}
	    break;
	    case 2:
	    if(serialData[3] == '0')
	      {
		*playGround = 0;
		step = 3;
	      }else if(serialData[3] == '1')
	      {
		*playGround = 1;
		step = 3;
	      }else{
		step = 0;
	      }
	      std::cout << "playground:" << *playGround << "\n";
	    break;
	    case 3:
		switch(serialData[5])
		{
			case '0':
                           *runStatus = 1;
                           step = 4;
			break;
			case '1':
                           *runStatus = 2;
                           step = 4;
			break;
			case '2':
                           *runStatus = 5;
			   step = 4;
			break;
			case '3':
                           *runStatus = 6;
                           step = 4;
			break;
			case '4':
                           *runStatus = 7;
                           step = 4;
			break;
			case '5':
                           *runStatus = 9;
                           step = 4;
			break;
			case '6':
                           *runStatus = 9;
                           step = 4;
			break;
			case '7':
                           *runStatus = 10;
                           step = 4;
			break;
                        case 's':
                           *runStatus = 8;
                           step = 4;
			break;
                        default:
                            step = 0;
                        break;
		}
	    break;
	    case 4:
                std::cout << "runStatus:" << *runStatus << "\n";
		my_serial->write("ok\r\n");
                wileFlag = 0;
	    break;
	    default:
	    break;
	  }
	}
}

void UpdateStatus(serial::Serial *my_serial,unsigned int *runStatus,int *playGround)
{
	uint8_t serialData[8];
	int step = 0;
	int wileFlag = 1;

	while(wileFlag)
	{
	  switch(step)
	  {
	    case 0:
	    my_serial->read(serialData,8);
	    step = 1;
	    break;
	    case 1:
	    if(serialData[0] == 'a' && serialData[1] == 't' && serialData[2] == '+')
		{
			step = 2;
		}
	    else{
		        wileFlag = 0;
		        std::cout << "wait...\n";
		}
	    break;
	    case 2:
	    if(serialData[3] == '0')
	      {
		*playGround = 0;
		step = 3;
	      }else if(serialData[3] == '1')
	      {
		*playGround = 1;
		step = 3;
	      }else{
		wileFlag = 0;
	      }
	      std::cout << "playground:" << *playGround << "\n";
	    break;
	    case 3:
		switch(serialData[5])
		{
			case '0':
                           *runStatus = BEFORE_DUNE_STAGE_1;
                           step = 4;
			break;
			case '1':
                           *runStatus = BEFORE_DUNE_STAGE_2;
                           step = 4;
			break;
			case '2':
                           *runStatus = BEFORE_GRASSLAND_STAGE_1;
			   step = 4;
			break;
			case '3':
                           *runStatus = BEFORE_GRASSLAND_STAGE_2;
                           step = 4;
			break;
			case '4':
                           *runStatus = UNDER_MOUNTAIN;
                           step = 4;
			break;
			case '5':
                           *runStatus = CLIMBING_MOUNTAIN;
                           step = 4;
			break;
			case '6':
                           *runStatus = CLIMBING_MOUNTAIN;
                           step = 4;
			break;
			case '7':
                           *runStatus = REACH_MOUNTAIN;
                           step = 4;
			break;
                        case 's':
                           *runStatus = 8;
                           step = 4;
			break;
                        default:
                            wileFlag = 0;
                        break;
		}
	    break;
	    case 4:
                std::cout << "runStatus:" << *runStatus << "\n";
		my_serial->write("ok\r\n");
                wileFlag = 0;
	    break;
	    default:
	    break;
	  }
	}
}
union dataUnion
{
	uint8_t dataChar[4];
	double data;
};

void SendDatas(serial::Serial *my_serial, int status, double frountData, double lateralDatas)
{
	dataUnion frountUnion;
	dataUnion lateralUnion;

	frountUnion.data = frountData;
	lateralUnion.data = lateralDatas;

	uint8_t allDatas[13];

	allDatas[0] = '\r';
	allDatas[1] = '\n';
	allDatas[11] = '\r';
	allDatas[12] = '\n';

	switch (status)
	{
	case 1:
		allDatas[2] = '0';
		break;
	case 2:
		allDatas[2] = '1';
		break;
	case 3:
		allDatas[2] = '1';
		break;
	case 4:
		allDatas[2] = '2';
		break;
	case 5:
		allDatas[2] = '2';
		break;
	case 6:
		allDatas[2] = '3';
		break;
	case 7:
		allDatas[2] = '4';
		break;
	case 9:
		allDatas[2] = '5';
		break;
	case 10:
		allDatas[2] = '7';
		break;
	}
	CopyData(frountUnion.dataChar, &allDatas[3], 4);
	CopyData(lateralUnion.dataChar, &allDatas[7], 4);

	my_serial->write(allDatas, 13);


}

void CopyData(uint8_t* origen, uint8_t* afterTreat, int size)
{
	for (size_t i = 0; i < size; i++)
	{
		*afterTreat = *origen;
		afterTreat++;
		origen++;
	}
}
#endif
