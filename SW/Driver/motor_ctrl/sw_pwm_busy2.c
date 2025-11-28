


void next_event(){
	if(RE){
		2x next
	}else{
		1x next
	}

	while(am_i_late()){
		2x next
	}
		
am_i_late(ps){
	now
}

{

	log_add(now, state)
	if(am_i_late()) {
		gpio()
	}else{
		log_late()
		next_event_I_am_not_late();
	}

}

