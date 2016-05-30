Fan :MultiOutUGen {

	*kr { arg normalized=0;
		^this.multiNew('control', normalized)
	}

	init { arg ... theInputs;
		inputs = theInputs;
		^this.initOutputs(2, rate);
	}


}

