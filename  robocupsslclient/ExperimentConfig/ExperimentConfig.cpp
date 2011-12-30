#include "ExperimentConfig.h"
#include "../VideoServer/Videoserver.h"
#include <boost/shared_ptr.hpp>
#include "boost/assert.hpp"
#include "../Logger/Logger.h"

ExperimentConfig::ExperimentConfig(std::string worldsFile,std::string resultsFile): video(Videoserver::getInstance()), log( getLoggerPtr ("app_debug") )
{
	this->resultsFile = resultsFile;
	std::ifstream file;
	file.open(worldsFile.c_str(),std::ios::in);
	if (!file){
		std::cout<<"ExperimentConfig: Plik z opisem światów \n";
		std::cout<<"("<<worldsFile<<") nie istnieje!"<<std::endl;
		exit(0);
	}

	//wczytywnanie światów z pliku
	std::string tmp; //bufor na dane
	bool complete = true;	//czy udało się wczytać cały opis świata zawarty między begin a end
	bool properties = false;
	bool speeds = true;

	const boost::regex reg_begin("\\s*(BEGIN)\\s+(\\S+)\\s*(.*)");	//opis wiersza z BEGIN + NAZWA EKSPERYMENTU
	//const boost::regex ex_name("\\s*(.*)");	//opis wiersza z BEGIN + NAZWA EKSPERYMENTU
	const boost::regex reg_end("\\s*(END)\\s*");

	std::string number_reg("\\s+([-+]?\\d+(\\.\\d+)?)"); //   \\s+([-]{0,1}\\d+)|
	const boost::regex
		reg_data("\\s*([a-zA-Z0-9_]+)"+number_reg+number_reg+number_reg+"\\s*");	//opis wiersza z danymi modelu

	const boost::regex
			reg_speed_data("\\s*(SPEED:)\\s*([a-zA-Z0-9_]+)"+number_reg+number_reg+"\\s*");	//opis wiersza z danymi predkosci

	const boost::regex empty_line("\\s*");

	const boost::regex
		reg_properties("\\s*TIME_LIMIT"+number_reg+"\\s*");	//opis wiersza z konfigiem swiata

	const boost::regex
			comment("^\\s*#.*");	//komentarz

	boost::shared_ptr<WorldDesc > w;

	LOG_INFO(this->log," [ExperimentConfig] start reading worlds descriptions ");

	try{
		while(getline(file,tmp)){
			boost::smatch what;


			if (boost::regex_match(tmp,empty_line)){
				continue;
			}

			if (boost::regex_match(tmp,comment)){
				continue;
			}


			if (boost::regex_match(tmp,what,reg_begin)){
				//std::cout<<tmp<<" pasuje do BEGIN!\n";
				if (complete) complete = false;
				else{
					std::string s("[ExperimentConfig] Plik z opisem światów\n("+worldsFile+") "+
							"jest niepoprawny (niedopasowane BEGIN-END)!\n");
					throw s;
				}
				std::string name = what[2];

				//std::string dynamic;
				//boost::smatch ww;
				//if (boost::regex_match(name,ww,ex_name)){
				//	name=ww[1];
				//	dynamic = ww[2];
				//}

				std::string dynamic = what[3];

				//LOG_INFO(this->log," name  "<<name << " !!! dynamic "<<dynamic);
				//exit(0);
				//sprawdzanie, czy taka nazwa juz nie wystapila

				std::vector<boost::shared_ptr<WorldDesc> >::iterator ii = worlds.begin();

				for(;ii!=worlds.end();ii++){
					if ((*ii)->getName() == name){
						std::string s("[ExperimentConfig] ERROR ("+worldsFile+
								"): świat o nazwie: "+ name +" już istnieje!\n");
						throw s;
					}
				}


				if (name.empty()){
					std::string s("[ExperimentConfig] Nie określono nazwy eksperymentu! \n("
							+ worldsFile +")\n");
					throw s;
				}
				else{
					w = boost::shared_ptr<WorldDesc>(new WorldDesc());
					w->setName(name);
					if( dynamic.compare("dynamic" )==0  ){
						LOG_INFO(this->log," !!!!![ExperimentConfig] dynamic experiment !!!!!!!!");
						w->setDynamic();
					}
					else
						LOG_INFO(this->log," !!!!![ExperimentConfig] static experiment !!!!!!!!");
				}

				properties = false;

				continue;
			}

			if (boost::regex_match(tmp,what, reg_speed_data)){
//				std::cout<<what[2]<<std::endl;	//model
//				std::cout<<what[3]<<std::endl;	//Vl
//				std::cout<<what[5]<<std::endl;	//Vr

				std::string name = what[2];
				std::string vxs  = what[3];
				std::string vys  = what[5];

				double vx = boost::lexical_cast<double>(vxs);
				double vy = boost::lexical_cast<double>(vys);

				w->addSpeedOrder(name,vx,vy);

				continue;
			}

			if (boost::regex_match(tmp,what, reg_properties)){
				//std::cout<<"Dopasowanie do prop\n";
				//std::cout<<what[1]<<std::endl;
				std::string t = what[1];
				double time = boost::lexical_cast<double>(t); //str2double(t);
				double ky = 2.495;
				time = time *ky;
				w->setTimeLimit(time);

				properties = true;
				continue;
			}

			if (boost::regex_match(tmp,reg_end)){
				if (!complete && speeds )complete = true;
				else{
					std::string s("[ExperimentConfig] Plik z opisem światów\n("+worldsFile+") "+
							"jest niepoprawny (niedopasowane BEGIN-END lub BEGIN_END-SPEEDS)!\n");
					throw s;
				}

				if (!properties)	//oznacza, że nie określono właściwości świata
				{
					std::string s("[ExperimentConfig] Nie określono właściwości świata ["+w->getName()+"]\n");
					throw s;
				}


				//std::cout<<"Dodano obiekt: \n";
				//w->display();

				worlds.push_back(w);
				continue;
			}



			if (boost::regex_match(tmp,what,reg_data)){
				//std::cout<<tmp<<" pasuje!\n";

				std::string model = what[1];
				std::string xs = what[2];
				std::string ys = what[4];
				std::string rot = what[6];
				double rot_ = boost::lexical_cast<double>(rot);	//str2double(rot);
				rot_ = rot_ * M_PI / 180.0;

				rot_ = convertAnglePI( rot_ - M_PI/2.0 );

				double kx = 2.257;
				double ky = 2.495;

				double x = boost::lexical_cast<double>(xs);
				double y = boost::lexical_cast<double>(ys);

				w->addObject( ( x - 0.15 )*kx, ( y - 0.15 )*ky,rot_,model);

				continue;
			}

			std::string s("[ExperimentConfig] ERROR: nie dopasowano linii: "+tmp+"\n");
			throw s;
		}

		if (!complete ){
			std::string s("[ExperimentConfig] Plik z opisem światów\n("+worldsFile+") "+
			"jest niepoprawny (niedopasowane BEGIN-END)!\n");
			throw s;
		}

		file.close();
	}
	catch(std::string s){
		file.close();
		std::cout<<s<<std::endl;
		exit(0);
	}

	LOG_INFO(this->log," [ExperimentConfig] ZAKOŃCZONO ");
	LOG_INFO(this->log," [ExperimentConfig] Wczytanych światów: "<<worlds.size() );

}

void ExperimentConfig::doExperiment( ){
	std::fstream results;

	std::vector<boost::shared_ptr<WorldDesc> >::iterator ii = worlds.end();
	double a, b, c;
	double kwant = 0.1;

	//video.updateData();


	results.open(resultsFile.c_str(), std::ios::in);
	if (!results) {
		std::cout<<"[staticEx] Plik wyjściowy ("<<resultsFile
				<<") nie istnieje, tworzenie\n";
		results.open(resultsFile.c_str(), std::ios::out);
		results
				<<"alfa1\talfa2\talfa3\tsrodowisko\tczasRozp\tczasZak\tukonczono\tczasSym"
				<<std::endl;
		//e = boost::shared_ptr<StaticExperiment>(new StaticExperiment());

		//USTAWIANIE WARUNKOW POCZATKOWYCH EKSPERYMENTU
		//a=kwant; b=0; c=0;
		a=kwant;
		b=0.0;
		c=0.0;
		ii = worlds.begin();
	} else {
		LOG_INFO( this->log," [staticEx] Wykryto plik wyjściowy ("<<resultsFile
				<<"), wznawianie eksperymentu" );

		std::string bufor, ostatni;
		while (getline(results, bufor)) {
			//std::cout<<bufor<<std::endl;
			if (bufor.length()>1)
				ostatni = bufor;
		}

		results.close();

		//string ostatni zawiera ostatni wiersz wykonanego eksperymentu
		//w zaleznosci od jego zawartosci zostanie zainicjowany eksperyment

		//std::cout<<ostatni<<std::endl;

		if (ostatni
				=="alfa1\talfa2\talfa3\tsrodowisko\tczasRozp\tczasZak\tukonczono\tczasSym") {
			//std::cout<<"Tylko pierwsza linia!"<<std::endl;
			a=kwant;
			b=0;
			c=0;
			ii = worlds.begin();
		} else {
			//0.1	0	0.9	eksperyment09	2008-Jul-17 18:22:45	2008-Jul-17 18:22:54	0	3.11


			std::string nr("([+]?\\d+(\\.\\d+)?)"); //   opisuje liczbę float (dodatnią!)
			std::string t("\\s");
			std::string
					date("(\\d{4}-[a-zA-Z]{2,4}-\\d{1,2}\\s\\d{2}:\\d{2}:\\d{2})");

			//const boost::regex 
			//reg2(nr + t + nr + t + nr + t + "([^\\s.]+)"+date+t+date+t+nr+t+nr);

			const boost::regex reg(nr + t + nr + t + nr + t + "(.+)"+t+date+t
					+date+t+nr+t+nr);

			boost::smatch what;
			if (boost::regex_match(ostatni, what, reg)) {
				//	std::cout<<"Dopasowano z sukcesem!\n";

				/*
				 std::cout<<"1: "<<what[1]<<std::endl;//a
				 std::cout<<"3: "<<what[3]<<std::endl;//b
				 std::cout<<"5: "<<what[5]<<std::endl;//c
				 std::cout<<"7: "<<what[7]<<std::endl;//nazwa
				 std::cout<<"8: "<<what[8]<<std::endl;//data
				 std::cout<<"9: "<<what[9]<<std::endl;//data
				 std::cout<<"10: "<<what[10]<<std::endl;//sukces
				 std::cout<<"12: "<<what[12]<<std::endl;//czas
				 */

				a = boost::lexical_cast<double>(what[1]); //this->str2double(what[1]);
				b = boost::lexical_cast<double>(what[3]); //this->str2double(what[3]);
				c = boost::lexical_cast<double>(what[5]); //this->str2double(what[5]);

				std::string nazwa_swiata = what[7];

				std::vector<boost::shared_ptr<WorldDesc> >::iterator tmp_ii = worlds.begin();
				for(;tmp_ii!=worlds.end();tmp_ii++) {
					if ((*tmp_ii)->getName()==nazwa_swiata) {
						ii = tmp_ii;
						ii++; //zeby przeskoczyc do nastepnego swiata!

						if (ii == worlds.end()) {
							ii = worlds.begin();
							c += kwant;
						}
						break;
					}
				}

				if (ii==worlds.end()) {
					LOG_ERROR( this->log," ERROR: Błąd w pliku wyjściowym ("<<resultsFile<<")" );
					LOG_ERROR( this->log," ERROR: na liście światów w pamięci nie istnieje świat o nazwie" );
					LOG_ERROR( this->log," ERROR: wczytanej z pliku (stary plik/zmieniono konfig swiata?)" );
					LOG_ERROR( this->log," ERROR: kontynuacja niemożliwa" );
					exit(0);
				}

			}
			else {
				LOG_ERROR( this->log,"ERROR: Błąd w pliku wyjściowym ("<<resultsFile<<")");
				LOG_ERROR( this->log,"ERROR: kontynuacja niemożliwa (błędna składnia?)");
				exit(0);
			}
		}

		LOG_INFO( this->log, "\nUWAGA: Wznawianie dla: a = "<<a<<" b = "<<b<<" c = "<< c <<" "<<(*ii)->getName() );
		results.open(resultsFile.c_str(),std::ios::out | std::ios::app);
		//results<<"alfa1\talfa2\talfa3\tsrodowisko\tczasRozp\tczasZak\tukonczono\tczasSym"<<std::endl;
	}

	for(;a<=1.0;a += kwant) {
		for(;b<=1.0;b += kwant) {
			for(;c<=1.0;c += kwant) {
				if (fabs(1.0 - (a+b+c)) < kwant/10.0) {
					//CVM::alfa1=a;
					//CVM::alfa2=b;
					//CVM::alfa3=c;
					
//					//zestaw nr siedem na potrzeby ustawiania czasu
//					CVM::alfa1=0.1;
//					CVM::alfa2=0.6;
//					CVM::alfa3=0.3;
					

					double alfa1=0.2;
					double alfa2=0.4;
					double alfa3=0.4;

					//results<<"ZESTAW WAG: "<<CVM::alfa1<<" "<<CVM::alfa2<<" "<<CVM::alfa3<<std::endl;
					//std::cout<<"\nZESTAW WAG: "<<CVM::alfa1<<" "<<CVM::alfa2<<" "
					//<<CVM::alfa3<<std::endl<<std::endl;
					//int num=0;
					for (; ii!=worlds.end(); ii++) {
						results<<alfa1<<"\t"<<alfa2<<"\t"<<alfa3<<"\t";

						LOG_INFO( this->log," EKSPERYMENT: "<<(*ii)->getName()<<" dynamic = "<<(*ii)->isDynamicWorld() );

						//Experiment e(results, *(*ii), isDynamic);
						Experiment e(results, *(*ii), (*ii)->isDynamicWorld() );
						//video.updateData();
						//Videoserver::data.display();	
						//std::ofstream file;
						//std::string fileName("tmp");
						//std::string tex(".tex");
						//fileName=fileName+boost::lexical_cast<std::string>(num++);
						//fileName=fileName+tex;
						//Videoserver::InitPrint(file,fileName,std::string("hmt_red0"));
						//int i=0;
						while (!e.finished(results)) {
							//video.updateData();
							//Videoserver::data.display();
							e.execute();
							//if(i++%2==0)
							//Videoserver::Print(file,std::string("hmt_red0"));
						}
						LOG_INFO( this->log," EKSPERYMENT: "<<(*ii)->getName()<<" finished" );
						results.flush();
						//Videoserver::FiniPrint(file,std::string("hmt_red0"));
						//Videoserver::data.display();

					}
					ii = worlds.begin();
				} //if				
			} //dla c
			c = 0.0;
		} //dla b
		b = 0.0;
	} //dla a


	//results<<"\n\n KONIEC\n";
	results.close();	
}

void ExperimentConfig::doEx()
{
	doExperiment( );
}

/*
void ExperimentConfig::doDynamicEx()
{
	doExperiment(true);
}
*/


ExperimentConfig::~ExperimentConfig()
{
}


/*
double ExperimentConfig::str2double(std::string s)
{
	double res;
	std::stringstream ss(s);
	ss >> res;
	return res;
}*/

void ExperimentConfig::display()
{
	std::vector<boost::shared_ptr<WorldDesc> >::iterator ii = worlds.begin();
	LOG_DEBUG( this->log,"[ExperimentConfig] Wczytane światy:");
	for(;ii!=worlds.end();ii++)
		(*ii)->display();
	LOG_DEBUG( this->log,"[ExperimentConfig] KONIEC");
}
