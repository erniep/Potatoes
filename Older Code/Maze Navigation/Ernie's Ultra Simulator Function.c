bool maze51[] = {true,false,false,true,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,false,true,true,false,false,false,true,false,true,true,true,false,true,true,false,false,true,true,false,true,false,true,true,false,false,true,true,false,true,false,true,false,true,false,true,false,true,false,false,true,true,false,true,false,false,true,false,true,true,false,true,true,false,false,true,false,true,false,true,false,true,false,true,false,true,true,false,true,true,false,false,true,false,true,false,true,false,true,false,false,false,false,true,false,false,false,true,false,true,false,true,false,true,true,false,false,true,true,false,true,false,true,false,false,false,false,true,false,false,false,true,true,true,false,true,false,true,false,true,false,true,false,false,true,true,true,true,true,false,false,true,true,true,false,false,true,true,true,true,false,false,false,true,false,true,false,false,true,true,true,false,true,false,true,false,true,false,true,false,true,false,true,true,true,false,false,true,true,true,false,true,true,true};

void naviSyst::callWallSensorsSimErnie() {
	unitCell* c = activeMap->getCurr();
	int n = c->getNum();
	c->setWall('N',maze51[((n-1)*4) + 0]);
	c->setWall('E',maze51[((n-1)*4) + 1]);
	c->setWall('S',maze51[((n-1)*4) + 2]);
	c->setWall('W',maze51[((n-1)*4) + 3]);

	c->setWallScan();
}

if(n == 48){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 41){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 40){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 39){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',true);	
	}
	else if(n == 38){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 37){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 34){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 33){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 32){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 31){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 30){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 27){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 26){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 25){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 24){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 23){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 20){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 19){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 18){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 17){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 16){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 13){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 12){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 11){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 10){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 9){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}