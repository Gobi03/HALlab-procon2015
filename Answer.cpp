//------------------------------------------------------------------------------
/// @file
/// @brief    HPCAnswer.hpp の実装 (解答記述用ファイル)
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCAnswer.hpp"
#include "HPCMath.hpp"
#include <string>
#include <limits.h>
#include <stack>

#include <bits/stdc++.h>
#define debug(x) cout << #x << " = " << x << endl

/// プロコン問題環境を表します。
namespace hpc {
  using namespace std;
  /// global value
  // chizu[y][x]
  // '#':wall, '.':load, 'S', 'G'
  int H;
  int W;
  char chizu[31][31];
  int dp_map[31][31];
  bool turn_start_flag;
  stack<int> routeStack;
  stack<int> tuminiStack;
  int nowCargo[10];
  int dp_sales[11][11];
  int weights[16];  // 荷物の重さ
  int perArray[16];  // 荷物の配達時間
  int periodPerCargo[4][10];  // [period][配達する番号]  あとで渡す
  int dpDis[16+1][16+1];  // すべての荷物間の距離をメモ化。index 総荷物数にスタート間の距離
  int pPCargoCnt[4];  // periodPerCargoのindexどこまで行ったか
  int periodPerWeight[4];


  //// functions

  /// stuck check
  // (# . z S G の５種類)
  // 'z' is already passed
  static bool stuckCheck(char vec, int x, int y){
    // are x and y in correct range?
    if(x < 0 || y < 0 || x >= W || y >= H)
      return false;
      
    char now = chizu[y][x];
    bool ans = false;

    if(now == '#')
      return false;
    else if(now == 'G')
      ans = true;
    else if(now == 'S' || now == 'z')
      return true;
    else  // '.'
      chizu[y][x] = 'z';
    
    bool res = false; // これ以降の調査の結果
    if(vec != 'r')  // to left
      res |= stuckCheck('l', x-1, y);
    if(vec != 'l')  // to right
      res |= stuckCheck('r', x+1, y);
    if(vec != 'd')  // to up
      res |= stuckCheck('u', x, y+1);
    if(vec != 'u')  // to down
      res |= stuckCheck('d', x, y-1);

    if(!(res || ans))
      chizu[y][x] = '#';

    return res || ans;
  }

  /// search shortest route
  // initialize dp_map(これは全範囲やる)
  void initDp_map(){
    for(int y=0; y < H; y++)
      for(int x=0; x < W; x++)
        dp_map[y][x] = INT_MAX;
  }
    
  // route A to B
  // AからBへの最短経路を地図に記す
  // fillRouteStackInit, fillRouteStackと組み合わせて使う
  // 通る順に番号ふるので、ゴールから降順に辿れば道わかる
  void routeAtoB(int Ax, int Ay, int Bx, int By, int nx, int ny, int cnt) {
    if(nx == Bx && ny == By && cnt < dp_map[ny][nx]) { // goal着いたら
      dp_map[By][Bx] = cnt;
      return;
    }
    else if(chizu[ny][nx] == '#')
      return;
    else {  // '.'
      if(cnt < dp_map[ny][nx]){
        dp_map[ny][nx] = cnt;

        routeAtoB(Ax, Ay, Bx, By, nx-1, ny, cnt+1);
        routeAtoB(Ax, Ay, Bx, By, nx+1, ny, cnt+1);
        routeAtoB(Ax, Ay, Bx, By, nx, ny-1, cnt+1);
        routeAtoB(Ax, Ay, Bx, By, nx, ny+1, cnt+1);
      }
      else{
        return;
      }
    }
  }

  /// routeを降順に辿ってrouteStackにその順序を積む
  // vec(0:left, 1:right, 2:down, 3:up)
  // vecはそのままsatckに積む（逆向きにたどっていることは織り込み済み）
  bool fillRouteStack(int x, int y, int cnt, int vec) {
    if(dp_map[y][x] != cnt)
      return false;
    else{ // dp_map[y][x] == cnt
      routeStack.push(vec);
      
      if(cnt == 0) // start着いた
        return true;
      else{
        bool flag = false;

        flag |= fillRouteStack(x-1, y, cnt-1, 1);
        if(!flag)
          flag |= fillRouteStack(x+1, y, cnt-1, 0);
        if(!flag)
          flag |= fillRouteStack(x, y-1, cnt-1, 3);
        if(!flag)
          flag |= fillRouteStack(x, y+1, cnt-1, 2);

        return true;
      }
    }
  }
  
  // fillRouteStackInitialに、ゴールの座標渡す
  void fillRouteStackInitial(int Gx, int Gy) {
    int cnt = dp_map[Gy][Gx];
    bool flag = false;

    flag |= fillRouteStack(Gx-1, Gy, cnt-1, 1);
    if(!flag)
      flag |= fillRouteStack(Gx+1, Gy, cnt-1, 0);
    if(!flag)
      flag |= fillRouteStack(Gx, Gy-1, cnt-1, 3);
    if(!flag)
      flag |= fillRouteStack(Gx, Gy+1, cnt-1, 2);

    return;
  }

  // fillNowCargo
  // nowCargoを昇順に(index 0から)埋めて、積み荷の個数を返す
  int fillNowCargo(ItemCollection aItemCollection, ItemGroup aIG){
    int cnt = 0;

    for (int i = 0; i < aItemCollection.count(); ++i) {
      if (aIG.hasItem(i)) {
        cnt++;
        nowCargo[cnt-1] = i;
      }
    }

    return cnt;
  }

  // searchAtoBDistance
  // AB間の最短距離を返す。
  // dp_mapをclearするので注意
  // list<int> salesman;
  int searchAtoBDistance(int Ax, int Ay, int Bx, int By){
    initDp_map();
    routeAtoB(Ax, Ay, Bx, By, Ax, Ay, 0);

    return dp_map[By][Bx];
  }

  // fillDp_sales
  // dp_salesのindexはnowCargoの引数に当たるもの
  // index cargoNumがスタートに当たる
  // int nowCargo[10];
  void fillDp_sales(int cargoNum, ItemCollection aIC, int Sx, int Sy){
    // 真ん中に斜線引く
    for(int i=0; i <= cargoNum; i++)
      for(int j=0; j <= cargoNum; j++)
        if(i == j)
          dp_sales[i][j] = 0;

    // 右上埋める
    for(int i=0; i < cargoNum; i++){
      for(int j=i+1; j <= cargoNum; j++){
        if(j == cargoNum){ // スタートに向かう
          int dx = aIC[nowCargo[i]].destination().x;
          int dy = aIC[nowCargo[i]].destination().y;
          dp_sales[i][j] = searchAtoBDistance(dx, dy, Sx, Sy);
        }
        else{ // 配達先間
          int ax = aIC[nowCargo[i]].destination().x;
          int ay = aIC[nowCargo[i]].destination().y;
          int bx = aIC[nowCargo[j]].destination().x;
          int by = aIC[nowCargo[j]].destination().y;

          dp_sales[i][j] = searchAtoBDistance(ax, ay, bx, by);
        }
      }
    }

    // 右上を左下に写す
    for(int i=0; i < cargoNum; i++)
      for(int j=i+1; j <= cargoNum; j++)
        dp_sales[j][i] = dp_sales[i][j];
  }




  // fillTuminiStack(Initの方を呼ぶ)
  // tuminiStackを全探索によって最小経路を通るように積む
  // nowCargo[10]
  // int dp_sales[11][11]
  // int searchAtoBDistance(int Ax, int Ay, int Bx, int By)
  bool dp_passed[10];
  list<int> fillTuminiStack(int cargoNum, int tumini_cnt, list<int> salesman, ItemCollection aIC, int fuel){
    if(tumini_cnt == cargoNum){  // 終了条件
      // 末尾に距離情報と入れ替えに燃料情報入れる
      int nowdis = salesman.back();
      salesman.pop_back();
      fuel += (nowdis + dp_sales[salesman.back()][cargoNum]) * 3;
      salesman.push_back(fuel);
      
      return salesman;
    }
    else{  // 継続条件
      int ans_cnt = INT_MAX;
      list<int> ans_list;
      
      for(int i=0; i < cargoNum; i++){
        if(!dp_passed[i]){
          dp_passed[i] = true;

          // 現在の距離保管しておく
          int nowdis = salesman.back();
          salesman.pop_back();
          int preCargo = salesman.back();

          // 今回分の情報付加
          salesman.push_back(i);
          salesman.push_back(nowdis + dp_sales[preCargo][i]);
          
          list<int> tmp =
            fillTuminiStack(cargoNum, tumini_cnt + 1, salesman, aIC, fuel + nowdis * weights[nowCargo[i]]);
          if(tmp.back() < ans_cnt){
            ans_cnt = tmp.back();
            ans_list = tmp;
          }

          // 終了処理
          dp_passed[i] = false;
          salesman.pop_back();
          salesman.pop_back();
          salesman.push_back(nowdis);
        }
      }

      return ans_list;
    }
  }

  void fillTuminiStackInit(int Sx, int Sy, int cargoNum, ItemCollection aIC){
    int min = INT_MAX;
    list<int> salesman;
    list<int> anslist;
    // dp_passed initialize
    for(int i=0; i < cargoNum; i++)
      dp_passed[i] = false;

    // fillDp_sales呼び出し(fillDp_sales(int cargoNum, ItemCollection& aIC, int Sx, int Sy))
    fillDp_sales(cargoNum, aIC, Sx, Sy);

    // fillTuminiStack呼び出し
    for(int i=0; i < cargoNum; i++){
      dp_passed[i] = true;
      
      /// first method call
      // cons
      salesman.push_back(i);
      salesman.push_back(dp_sales[cargoNum][i]);
      
      list<int> tmp = fillTuminiStack(cargoNum, 1, salesman, aIC, salesman.back() * weights[nowCargo[i]]);
      if(tmp.back() < min){
        min = tmp.back();
        tmp.pop_back();
        anslist = tmp;
      }
      
      salesman.clear();
      dp_passed[i] = false;
    }

    // tuminiStackに積み込み
    // anslistはすでにdistance排除済み
    list<int>::reverse_iterator it = anslist.rbegin();
    while( it != anslist.rend() ) {
      tuminiStack.push(nowCargo[*it]);
      it++;
    }
  }






  //------------------------------------------------------------------------------
  /// 各ステージ開始時に呼び出されます。
  ///
  /// ここで、各ステージに対して初期処理を行うことができます。
  ///
  /// @param[in] aStage 現在のステージ。

  // int turn = 0;
  void Answer::Init(const Stage& aStage)
  {
    // // turn display(debug code)
    // cout << "-----------------" << endl;
    // debug(turn);
    // cout << "-----------------" << endl;
    // turn++;

    turn_start_flag = true;

    ItemCollection aIC = aStage.items();
    Field aField = aStage.field();
    int sx = aField.officePos().x;
    int sy = aField.officePos().y;
    H = aField.height();
    W = aField.width();

    // fill a weights and a perlist
    for (int i = 0; i < aIC.count(); ++i) {
      weights[i] = aIC[i].weight();
      perArray[i] = aIC[i].period();
    }
    
    // make a map(これは外周もやる)
    for(int y=0; y < H; y++){
      for(int x=0; x < W; x++){
        if(aField.isWall(x, y))
          chizu[y][x] = '#';
        else
          chizu[y][x] = '.';
      }
    }

    // write a start and goals points
    chizu[sy][sx] = 'S';
    for (int i = 0; i < aIC.count(); ++i) {
      Item aItem = aIC[i];
      Pos aPos = aItem.destination();

      int x = aPos.x;
      int y = aPos.y;
      chizu[y][x] = 'G';
    }

    // delete stuck route
    stuckCheck('l', sx-1, sy);
    stuckCheck('r', sx+1, sy);
    stuckCheck('d', sx, sy-1);
    stuckCheck('u', sx, sy+1);


    // recover 'z', 'S', 'G' to '.'
    for(int y=1; y < H-1; y++)
      for(int x=1; x < W-1; x++)
        if(chizu[y][x] != '#')
          chizu[y][x] = '.';


    //// ver1.2から追加
    /// int perArray[16];
    /// int periodPerCargo[4][10];
    /// int pPCargoCnt[4]
    /// int dpDis[16+1][16+1];
    /// int periodPerWeight[4];

    // initialize pPCargoCnt[4] and periodPerWeight[4]
    for(int i=0; i < 4; i++){
      pPCargoCnt[i] = 0;
      periodPerWeight[i] = 0;
    }
    // fill periodPerCargo and periodPerWeight
    for(int i=0; i < aIC.count(); i++){
      int pertmp = perArray[i];
      if(pertmp != -1){
        periodPerCargo[pertmp][pPCargoCnt[pertmp]] = i;
        pPCargoCnt[pertmp]++;
        
        periodPerWeight[pertmp] += weights[i];
      }
    }
    
    // fill dpDis(すべての目的地 + Start間の距離を算出)
    // 真ん中に斜線引く
    for(int i=0; i <= aIC.count(); i++)
      for(int j=0; j <= aIC.count(); j++)
        if(i == j)
          dpDis[i][j] = 0;
    // 右上埋める
    for(int i=0; i < aIC.count(); i++){
      for(int j=i+1; j <= aIC.count(); j++){
        if(j == aIC.count()){ // スタートに向かう
          int dx = aIC[i].destination().x;
          int dy = aIC[i].destination().y;
          dpDis[i][j] = searchAtoBDistance(dx, dy, sx, sy);
        }
        else{ // 配達先間
          int ax = aIC[i].destination().x;
          int ay = aIC[i].destination().y;
          int bx = aIC[j].destination().x;
          int by = aIC[j].destination().y;

          dpDis[i][j] = searchAtoBDistance(ax, ay, bx, by);
        }
      }
    }
    // 右上を左下に写す
    for(int i=0; i < aIC.count(); i++)
      for(int j=i+1; j <= aIC.count(); j++)
        dpDis[j][i] = dpDis[i][j];


    // per = -1の荷物を割り振る
    for(int i=0; i < aIC.count(); i++){ // i は(per = -1)の荷物のindex
      int pertmp = perArray[i];
      if(pertmp == -1){
        int ansper = -1;
        int minDis = INT_MAX;
        for(int j=0; j < 4; j++){ // j は検査するperiodのindex
          if(periodPerWeight[j] + weights[i] <= 14){  // この数字が積み荷の総重量の限界表す
            if(pPCargoCnt[j] == 0)
              {  // そのピリオドに荷物がない場合(startで見る)
                int tmp = dpDis[i][aIC.count()];
                if(tmp < minDis){
                  minDis = tmp;
                  ansper = j;
                }
              }
            else
              {  
                for(int k=0; k < pPCargoCnt[j]; k++){
                  int tmp = dpDis[i][periodPerCargo[j][k]];
                  if(tmp < minDis){
                    minDis = tmp;
                    ansper = j;
                  }
                }
              }
          }
        }

        periodPerCargo[ansper][pPCargoCnt[ansper]] = i;
        pPCargoCnt[ansper]++;
        periodPerWeight[ansper] += weights[i];
      }
    }
  }




  //------------------------------------------------------------------------------
  /// 各配達時間帯開始時に呼び出されます。
  ///
  /// ここで、この時間帯に配達する荷物をトラックに積み込みます。
  /// どの荷物をトラックに積み込むかを決めて、引数の aItemGroup に対して
  /// setItem で荷物番号を指定して積み込んでください。
  ///
  /// @param[in] aStage 現在のステージ。
  /// @param[in] aItemGroup 積み荷グループ。
  /// @param[in] aItemCollection すべての荷物。
  
  // tuminiStack
  void Answer::InitPeriod(const Stage& aStage, ItemGroup& aItemGroup)
  {
    // perは現在のピリオド
    int per = aStage.period();
    ItemCollection aItemCollection = aStage.items();

    turn_start_flag = true;

    // periodPerCargo[4][10]から積み込み
    for(int i=0; i < pPCargoCnt[per]; i++)
      aItemGroup.addItem(periodPerCargo[per][i]);

    // nowCargoを記入
    int cargoNum = fillNowCargo(aItemCollection, aItemGroup);

    // tuminiStack埋める
    // fillTuminiStackInit(int Sx, int Sy, int cargoNum, ItemCollection aIC)
    int Sx = aStage.field().officePos().x;
    int Sy = aStage.field().officePos().y;

    fillTuminiStackInit(Sx, Sy, cargoNum, aItemCollection);
  }



  //------------------------------------------------------------------------------
  /// 各ターンでの動作を返します。
  ///
  /// @param[in] aStage 現在ステージの情報。
  ///
  /// @return これから行う動作を表す Action クラス。

  // chizu[y][x]
  // routeAtoB(int Ax, int Ay, int Bx, int By, int nx, int ny, int cnt)
  // initDp_map()

  Action Answer::GetNextAction(const Stage& aStage)
  {
    // turn change process
    if(turn_start_flag){
      turn_start_flag = false;
    }
    
    // values
    Action aAction;  // answer
    Field aField = aStage.field();
    ItemCollection aItemCollection = aStage.items(); // all items
    Truck aTruck = aStage.truck(); // truck
    // int period = aStage.period();  // int (current period) 1〜4
    // ItemGroup aIG = aTruck.itemGroup();  // 積み荷
    int sx = aField.officePos().x;  // 営業所
    int sy = aField.officePos().y;
    int nx = aTruck.pos().x;  // 現在地
    int ny = aTruck.pos().y;

    // 移動処理
    if(tuminiStack.empty() && routeStack.empty()){  // スタートに戻る準備
      initDp_map(); // initialize
      routeAtoB(nx, ny, sx, sy, nx, ny, 0);
      fillRouteStackInitial(sx, sy);
    }
    else if(routeStack.empty()){  // prepare to go next goal
      initDp_map(); // initialize
      
      Item nItem = aItemCollection[tuminiStack.top()];
      tuminiStack.pop();
      int gx = nItem.destination().x;
      int gy = nItem.destination().y;
      routeAtoB(nx, ny, gx, gy, nx, ny, 0);
      fillRouteStackInitial(gx, gy);
    }

    // set action
    aAction = static_cast<Action>(routeStack.top());
    routeStack.pop();

    return aAction;
  }



  //------------------------------------------------------------------------------
  /// 各配達時間帯終了時に呼び出されます。
  ///
  /// ここで、この時間帯の終了処理を行うことができます。
  ///
  /// @param[in] aStage 現在のステージ。
  /// @param[in] aStageState 結果。Playingならこの時間帯の配達完了で、それ以外なら、何らかのエラーが発生した。
  /// @param[in] aCost この時間帯に消費した燃料。エラーなら0。
  void Answer::FinalizePeriod(const Stage& aStage, StageState aStageState, int aCost)
  {
    if (aStageState == StageState_Failed) {
      // 失敗したかどうかは、ここで検知できます。
    }
  }



  //------------------------------------------------------------------------------
  /// 各ステージ終了時に呼び出されます。
  ///
  /// ここで、各ステージに対して終了処理を行うことができます。
  ///
  /// @param[in] aStage 現在のステージ。
  /// @param[in] aStageState 結果。Completeなら配達完了で、それ以外なら、何らかのエラーが発生した。
  /// @param[in] aScore このステージで獲得したスコア。エラーなら0。
  void Answer::Finalize(const Stage& aStage, StageState aStageState, int aScore)
  {
    // original process
    if (aStageState == StageState_Failed) {
      // 失敗したかどうかは、ここで検知できます。
    }
    else if (aStageState == StageState_TurnLimit) {
      // ターン数オーバーしたかどうかは、ここで検知できます。
    }

    // my_process
    while(!routeStack.empty())
      routeStack.pop();
    while(!tuminiStack.empty())
      tuminiStack.pop();
    

    // // Score表示
    // cout << "Score: " << aScore << endl;
  }
}

//------------------------------------------------------------------------------
// EOF
