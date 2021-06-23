should remain
switch(name){
    // <FACTORY_GENERATOR_INSTANCE_CREATION>
	if(name == "TestItem4"){
		return new TestItem4(p1, p2);
	}
	else if(name == "TestItem5"){
	    return new TestItem5(p1, p2);
	}
    // <FACTORY_GENERATOR_INSTANCE_CREATION/>
    else{
        return null;
    }
}
should remain

std::string FilterFactory::GetFilterList() {
    // <FACTORY_GENERATOR_ITEMS_LIST>
	return 'TestItem1;TestItem2';
    // <FACTORY_GENERATOR_ITEMS_LIST/>
}
should remain