should remain
switch(name){
    // <FACTORY_GENERATOR_INSTANCE_CREATION>
	if(name == "TestItem3000"){
		return new TestItem3000(p1, p2);
	}
	else if(name == "TestItem30"){
	    return new TestItem30(p1, p2);
	}
    // <FACTORY_GENERATOR_INSTANCE_CREATION/>
    else{
        return null;
    }
}
should remain