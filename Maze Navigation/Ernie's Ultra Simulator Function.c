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