import java.util.Random;
import java.util.Scanner;

public class Main {
    public static void main(String[] args){
        Scanner scanner = new Scanner(System.in);

        String[] symbols = {"🍇", "🍉", "🥭", "🍑" ,"🍅"};
        String play="y";
        int balance = 0;
        int bet;

        System.out.println("*************************");
        System.out.println(" Welcome to SLOT MACHINE ");
        System.out.println(" Symbols: 🍇 🍉 🥭 🍑 🍅");
        System.out.println("*************************");

        do {
            System.out.print("Enter your Bet Amount: ");
            bet = scanner.nextInt();
            scanner.nextLine();
            if (bet > balance) {
                balance=lowBalance(balance);
            } else if (bet<=0) {
                System.out.println("Enter a valid bet..");
                continue;
            } else{
                balance-=bet;
                System.out.println("Spinning...");
                balance = spinRow(bet,balance,symbols);
            }
            System.out.println("Your Balance: "+ balance);
            System.out.print("Want to continue with the game?(y/n) ");
            play = scanner.nextLine();
        }while (play.equals("y"));
        System.out.println("Thank You!!!");
        scanner.close();
    }
    static int lowBalance(int balance){
        Scanner scanner = new Scanner(System.in);

        String deposit;
        int depAmount;
        boolean dep = true;
        System.out.println("Insufficient Balance!!!");
        do{
            System.out.print("Want to deposit amount?(y/n) ");
            deposit = scanner.nextLine();
            if (deposit.equals("y")) {
                System.out.print("Enter Amount to deposit: ");
                depAmount = scanner.nextInt();
                scanner.nextLine();
                if(depAmount>0){
                    balance += depAmount;
                    return balance;
                }
                else {
                    System.out.println("Enter a valid amount..");
                    continue;
                }
            } else if (deposit.equals("n")) {
                return balance;
            }
            else {
                System.out.println("Enter a valid option!!");
            }
        }while (dep);
        return balance;
    }
    static int spinRow(int bet,int balance, String[] symbols){
        int payOut=0;
        Random random = new Random();
        String[] rowSymbols= new String[3];

        for(int i=0;i<rowSymbols.length;i++){
            rowSymbols[i]=symbols[random.nextInt(5)];
        }
        System.out.println("****************");
        System.out.println(" "+String.join(" | ",rowSymbols)+" ");
        System.out.println("****************");

        if(rowSymbols[0].equals(rowSymbols[1]) && rowSymbols[1].equals(rowSymbols[2])){
            switch (rowSymbols[0]){
                case("🍇"):
                    payOut+=10;
                    break;
                case("🍉"):
                    payOut+=20;
                    break;
                case("🥭"):
                    payOut+=30;
                    break;
                case("🍅"):
                    payOut+=40;
                    break;
                case("🍑"):
                    payOut+=50;
                    break;
                default:
                    break;
            }
            System.out.println("You WON!!!");
            System.out.println("Yor price: "+ (payOut+bet));
            balance+=payOut+bet;
        }
        else if (rowSymbols[0].equals(rowSymbols[1])) {
            switch (rowSymbols[0]){
                case("🍇"):
                    payOut+=5;
                    break;
                case("🍉"):
                    payOut+=10;
                    break;
                case("🥭"):
                    payOut+=15;
                    break;
                case("🍅"):
                    payOut+=20;
                    break;
                case("🍑"):
                    payOut+=30;
                    break;
                default:
                    break;
            }
            System.out.println("You WON!!!");
            System.out.println("Yor price: "+ (payOut+bet));
            balance+=payOut+bet;

        }
        else if (rowSymbols[1].equals(rowSymbols[2])) {
            switch (rowSymbols[1]){
                case("🍇"):
                    payOut+=5;
                    break;
                case("🍉"):
                    payOut+=10;
                    break;
                case("🥭"):
                    payOut+=15;
                    break;
                case("🍅"):
                    payOut+=20;
                    break;
                case("🍑"):
                    payOut+=30;
                    break;
                default:
                    break;
            }
            System.out.println("You WON!!!");
            System.out.println("Yor price: "+ (payOut+bet));
            balance+=payOut+bet;
        }
        else {
            System.out.println("SORRY!! YOU LOST!!");
        }
        return balance;
    }
}