should remain
switch(name){
    // <FACTORY_GENERATOR_INSTANCE_CREATION>
    case 'AdaptiveThreshold':
		return new AdaptiveThreshold(globalParams);
	case 'BackgroundSubstract':
		return new BackgroundSubstract(globalParams);
    // <FACTORY_GENERATOR_INSTANCE_CREATION/>
    default:
        return null;
}
should remain

std::string FilterFactory::GetFilterList() {
    // <FACTORY_GENERATOR_ITEMS_LIST>
	return 'Stuff;Other';
    // <FACTORY_GENERATOR_ITEMS_LIST/>
}
should remain