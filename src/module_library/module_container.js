// Definition of module container

class ModuleContainer(){
	constructor(){
		this.libraries = {}
	}

	addModule(library, name, module_class){
		if(! this.libraries.library) {
			console.log('New library');
			this.libraries[library] = {};
		}
		this.libraries[library][name] = module_class;
	}
}

var module_container = new ModuleContainer()
