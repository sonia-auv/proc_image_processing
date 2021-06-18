should remain
switch(name){
    // <FACTORY_GENERATOR_INSTANCE_CREATION>
	case 'TestItem1':
		return new TestItem1(p1, p2);
	case 'TestItem2':
		return new TestItem2(p1, p2);
    // <FACTORY_GENERATOR_INSTANCE_CREATION/>
    default:
        return null;
}
should remain

std::string FilterFactory::GetFilterList() {
    // <FACTORY_GENERATOR_ITEMS_LIST>
	return 'TestItem1;TestItem2';
    // <FACTORY_GENERATOR_ITEMS_LIST/>
}
should remain