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